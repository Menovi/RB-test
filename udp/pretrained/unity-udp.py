''''
#version 1

import argparse, os, time, socket, struct, threading
import numpy as np
import mujoco, mujoco_viewer
from tqdm import tqdm
from collections import deque
import torch

from utils.motion_lib import MotionLib

# --- UDP Listener Thread ---
class UnityReceiver(threading.Thread):
    def __init__(self, ip="127.0.0.1", port=5005):
        super(UnityReceiver, self).__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.01)
        self.current_mode = 0 
        self.new_data_available = False 
        self.running = True

    def run(self):
        print(f"UDP Receiver active on port 5005.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(120) 
                floats = struct.unpack('f' * 30, data)
                new_mode = int(floats[29]) 
                
                # Only signal a change if the ID is actually different
                if new_mode != self.current_mode:
                    self.current_mode = new_mode
                    self.new_data_available = True 
            except socket.timeout:
                continue

# UPDATED: ID 0 is Idle, ID 1 is Walk Stand
MOTION_MAP = {
    0: "walk_stand.pkl", # Virtual Idle override in _get_mimic_obs
    1: "walk_stand.pkl",
    2: "basic_walk.pkl",
    3: "dance.pkl",
    4: "squat.pkl",
    5: "kick_walk.pkl",
    6: "dance_waltz.pkl",
    7: "crouchwalk_stand.pkl",
    8: "airkick_stand.pkl"
}

@torch.jit.script
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def euler_from_quaternion(quat_angle):
    x = quat_angle[:,0]; y = quat_angle[:,1]; z = quat_angle[:,2]; w = quat_angle[:,3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = torch.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = torch.clip(t2, -1, 1)
    pitch_y = torch.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = torch.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z 

def quatToEuler(quat):
    eulerVec = np.zeros(3)
    qw, qx, qy, qz = quat
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    eulerVec[0] = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    eulerVec[1] = np.arcsin(np.clip(sinp, -1, 1))
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    eulerVec[2] = np.arctan2(siny_cosp, cosy_cosp)
    return eulerVec

class HumanoidEnv:
    def __init__(self, policy_path, motion_path, robot_type="g1", device="cuda"):
        self.robot_type = robot_type
        self.device = device
        self.motion_path = motion_path
        
        model_path = "assets/robots/g1/g1.xml"
        self.stiffness = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40, 150, 150, 150, 40, 40, 40, 40, 40, 40, 40, 40])
        self.damping = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5])
        self.default_dof_pos = np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0, -0.2, 0.0, 0.0, 0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 1.2, 0.0, -0.4, 0.0, 1.2])
        self.torque_limits = np.array([88, 139, 88, 139, 50, 50, 88, 139, 88, 139, 50, 50, 88, 50, 50, 25, 25, 25, 25, 25, 25, 25, 25])
        
        self.num_actions = 23
        self.num_dofs = 23
        self.sim_dt = 0.001
        self.sim_decimation = 20
        self.control_dt = self.sim_dt * self.sim_decimation
        self.action_scale = 0.5
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        self.policy_jit = torch.jit.load(policy_path, map_location=self.device)
        self._motion_lib = MotionLib(self.motion_path, self.device)
        self.tar_obs_steps = torch.tensor([1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95], device=self.device, dtype=torch.int)
        
        self.n_proprio = 3 + 2 + 3*self.num_actions
        self.proprio_history_buf = deque(maxlen=20)
        for _ in range(20): self.proprio_history_buf.append(np.zeros(self.n_proprio))
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)

        self.receiver = UnityReceiver()
        self.receiver.start()
        # Start in Mode 0 (Idle)
        self.active_mode = 0 

    def reset_robot(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[-self.num_dofs:] = self.default_dof_pos
        mujoco.mj_forward(self.model, self.data)
        print("[SYSTEM] Robot Reset to Standing Pose.")

    def _get_mimic_obs(self, curr_time_step):
        num_steps = len(self.tar_obs_steps)
        motion_times = torch.tensor([curr_time_step * self.control_dt], device=self.device).unsqueeze(-1)
        obs_motion_times = (self.tar_obs_steps * self.control_dt + motion_times).flatten()
        motion_ids = torch.zeros(num_steps, dtype=torch.int, device=self.device)
        
        root_pos, root_rot, root_vel, root_ang_vel, dof_pos, _ = self._motion_lib.calc_motion_frame(motion_ids, obs_motion_times)
        
        # IDLE OVERRIDE: If mode is 0, zero out reference movement
        if self.active_mode == 0:
            root_vel[:] = 0
            root_ang_vel[:] = 0
            dof_pos[:] = torch.from_numpy(self.default_dof_pos).to(self.device)

        roll, pitch, yaw = euler_from_quaternion(root_rot)
        mimic_obs_buf = torch.cat((
            root_pos.reshape(1, num_steps, 3)[..., 2:3],
            roll.reshape(1, num_steps, 1), pitch.reshape(1, num_steps, 1),
            quat_rotate_inverse(root_rot, root_vel).reshape(1, num_steps, 3),
            quat_rotate_inverse(root_rot, root_ang_vel).reshape(1, num_steps, 3)[..., 2:3],
            dof_pos.reshape(1, num_steps, -1),
        ), dim=-1)
        return mimic_obs_buf.reshape(1, -1).detach().cpu().numpy().squeeze()

    def run(self):
        step_counter = 0
        pd_target = self.default_dof_pos.copy()
        start_time = time.time()
        
        # Initial State Reset
        self.reset_robot()
        
        while self.viewer.is_alive:
            # 1. Improved Handshake Logic
            if self.receiver.new_data_available:
                target_mode = self.receiver.current_mode
                motion_file = os.path.join("assets/motions", MOTION_MAP.get(target_mode, "walk_stand.pkl"))
                
                if os.path.exists(motion_file):
                    print(f"\n[SYSTEM] Loading: {os.path.basename(motion_file)}")
                    self._motion_lib = MotionLib(motion_file, self.device)
                    self.active_mode = target_mode
                    
                    # Reset physical state when switching to IDLE
                    if self.active_mode == 0:
                        self.reset_robot()
                    
                    step_counter = 0 # Restart phase for the new motion
                    # Critical: Signal the receiver that we are done loading
                    self.receiver.new_data_available = False 
                else:
                    print(f"Error: {motion_file} missing.")
                    self.receiver.new_data_available = False

            # 2. Control Rate (50Hz)
            if step_counter % self.sim_decimation == 0:
                dof_pos = self.data.qpos.astype(np.float32)[-self.num_dofs:]
                dof_vel = self.data.qvel.astype(np.float32)[-self.num_dofs:]
                quat = self.data.sensor('orientation').data.astype(np.float32)
                ang_vel = self.data.sensor('angular-velocity').data.astype(np.float32)
                
                mimic_obs = self._get_mimic_obs(step_counter // self.sim_decimation)
                rpy = quatToEuler(quat)
                obs_dof_vel = dof_vel.copy()
                obs_dof_vel[[4, 5, 10, 11]] = 0.
                
                obs_prop = np.concatenate([
                    ang_vel * 0.25,
                    rpy[:2],
                    (dof_pos - self.default_dof_pos) * 1.0,
                    obs_dof_vel * 0.05,
                    self.last_action,
                ])
                
                obs_hist = np.array(self.proprio_history_buf).flatten()
                obs_buf = np.concatenate([mimic_obs, obs_prop, obs_hist])
                obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    raw_action = self.policy_jit(obs_tensor).cpu().numpy().squeeze()
                
                self.last_action = raw_action.copy()
                scaled_actions = np.clip(raw_action, -10., 10.) * self.action_scale
                pd_target = scaled_actions + self.default_dof_pos
                self.proprio_history_buf.append(obs_prop)

            # 3. Physics Step (1000Hz)
            dof_pos_p = self.data.qpos.astype(np.float32)[-self.num_dofs:]
            dof_vel_p = self.data.qvel.astype(np.float32)[-self.num_dofs:]
            torque = (pd_target - dof_pos_p) * self.stiffness - dof_vel_p * self.damping
            self.data.ctrl = np.clip(torque, -self.torque_limits, self.torque_limits)
            
            mujoco.mj_step(self.model, self.data)
            
            # 4. Timing & Sync
            step_counter += 1
            expected_time = step_counter * self.sim_dt
            actual_time = time.time() - start_time
            if actual_time < expected_time:
                time.sleep(max(0, expected_time - actual_time))
            
            if step_counter % self.sim_decimation == 0:
                self.viewer.render()

if __name__ == "__main__":
    jit_policy_pth = "assets/pretrained_checkpoints/pretrained.pt"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # Initial motion file for boot-up
    init_file = os.path.join("assets/motions", "walk_stand.pkl")
    env = HumanoidEnv(policy_path=jit_policy_pth, motion_path=init_file, device=device)
    env.run()
'''

'''
#repeated refresh of mujoco window but stops after clicking

import argparse, os, time, socket, struct, threading
import numpy as np
import mujoco, mujoco_viewer
from tqdm import tqdm
from collections import deque
import torch

from utils.motion_lib import MotionLib

# --- UDP Listener Thread ---
class UnityReceiver(threading.Thread):
    def __init__(self, ip="127.0.0.1", port=5005):
        super(UnityReceiver, self).__init__()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.sock.settimeout(0.01)
        self.current_mode = 0 
        self.running = True

    def run(self):
        print(f"UDP Receiver active on port 5005.")
        while self.running:
            try:
                data, addr = self.sock.recvfrom(120) 
                floats = struct.unpack('f' * 30, data)
                new_mode = int(floats[29]) 
                
                # Keep the mode updated; the main loop will check against active_mode
                self.current_mode = new_mode
            except socket.timeout:
                continue

# Mapping matching your Unity IDs
MOTION_MAP = {
    0: "walk_stand.pkl",
    1: "walk_stand.pkl",
    2: "basic_walk.pkl",
    3: "dance.pkl",
    4: "squat.pkl",
    5: "kick_walk.pkl",
    6: "dance_waltz.pkl",
    7: "crouchwalk_stand.pkl",
    8: "airkick_stand.pkl"
}

@torch.jit.script
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def euler_from_quaternion(quat_angle):
    x = quat_angle[:,0]; y = quat_angle[:,1]; z = quat_angle[:,2]; w = quat_angle[:,3]
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = torch.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = torch.clip(t2, -1, 1)
    pitch_y = torch.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = torch.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z 

def quatToEuler(quat):
    eulerVec = np.zeros(3)
    qw, qx, qy, qz = quat
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
    eulerVec[0] = np.arctan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (qw * qy - qz * qx)
    eulerVec[1] = np.arcsin(np.clip(sinp, -1, 1))
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    eulerVec[2] = np.arctan2(siny_cosp, cosy_cosp)
    return eulerVec

class HumanoidEnv:
    def __init__(self, policy_path, motion_path, robot_type="g1", device="cuda"):
        self.robot_type = robot_type
        self.device = device
        self.motion_path = motion_path
        
        model_path = "assets/robots/g1/g1.xml"
        self.stiffness = np.array([100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40, 150, 150, 150, 40, 40, 40, 40, 40, 40, 40, 40])
        self.damping = np.array([2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5])
        self.default_dof_pos = np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0, -0.2, 0.0, 0.0, 0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 1.2, 0.0, -0.4, 0.0, 1.2])
        self.torque_limits = np.array([88, 139, 88, 139, 50, 50, 88, 139, 88, 139, 50, 50, 88, 50, 50, 25, 25, 25, 25, 25, 25, 25, 25])
        
        self.num_actions = 23
        self.num_dofs = 23
        self.sim_dt = 0.001
        self.sim_decimation = 20
        self.control_dt = self.sim_dt * self.sim_decimation
        self.action_scale = 0.5
        
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)
        
        self.policy_jit = torch.jit.load(policy_path, map_location=self.device)
        self._motion_lib = MotionLib(self.motion_path, self.device)
        self.tar_obs_steps = torch.tensor([1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95], device=self.device, dtype=torch.int)
        
        self.n_proprio = 3 + 2 + 3*self.num_actions
        self.proprio_history_buf = deque(maxlen=20)
        for _ in range(20): self.proprio_history_buf.append(np.zeros(self.n_proprio))
        self.last_action = np.zeros(self.num_actions, dtype=np.float32)

        self.receiver = UnityReceiver()
        self.receiver.start()
        # Start in Mode 0 (Idle)
        self.active_mode = 0 

    def reset_robot(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[-self.num_dofs:] = self.default_dof_pos
        mujoco.mj_forward(self.model, self.data)
        print("[SYSTEM] Robot Reset to Standing Pose.")

    def _get_mimic_obs(self, curr_time_step):
        num_steps = len(self.tar_obs_steps)
        motion_times = torch.tensor([curr_time_step * self.control_dt], device=self.device).unsqueeze(-1)
        obs_motion_times = (self.tar_obs_steps * self.control_dt + motion_times).flatten()
        motion_ids = torch.zeros(num_steps, dtype=torch.int, device=self.device)
        
        root_pos, root_rot, root_vel, root_ang_vel, dof_pos, _ = self._motion_lib.calc_motion_frame(motion_ids, obs_motion_times)
        
        # IDLE OVERRIDE: Force velocity to 0 and pose to default if in Mode 0
        if self.active_mode == 0:
            root_vel[:] = 0
            root_ang_vel[:] = 0
            dof_pos[:] = torch.from_numpy(self.default_dof_pos).to(self.device)

        roll, pitch, yaw = euler_from_quaternion(root_rot)
        mimic_obs_buf = torch.cat((
            root_pos.reshape(1, num_steps, 3)[..., 2:3],
            roll.reshape(1, num_steps, 1), pitch.reshape(1, num_steps, 1),
            quat_rotate_inverse(root_rot, root_vel).reshape(1, num_steps, 3),
            quat_rotate_inverse(root_rot, root_ang_vel).reshape(1, num_steps, 3)[..., 2:3],
            dof_pos.reshape(1, num_steps, -1),
        ), dim=-1)
        return mimic_obs_buf.reshape(1, -1).detach().cpu().numpy().squeeze()

    def run(self):
        step_counter = 0
        pd_target = self.default_dof_pos.copy()
        start_time = time.time()
        
        self.reset_robot()
        
        while self.viewer.is_alive:
            # --- Handshake: Check if Unity mode differs from Simulation mode ---
            if self.receiver.current_mode != self.active_mode:
                target_mode = self.receiver.current_mode
                motion_file = os.path.join("assets/motions", MOTION_MAP.get(target_mode, "walk_stand.pkl"))
                
                if os.path.exists(motion_file):
                    print(f"\n[SYSTEM] Loading: {os.path.basename(motion_file)}")
                    self._motion_lib = MotionLib(motion_file, self.device)
                    # Update active_mode AFTER the file is successfully loaded
                    self.active_mode = target_mode
                    
                    if self.active_mode == 0:
                        self.reset_robot()
                    
                    step_counter = 0 
                else:
                    print(f"Error: {motion_file} missing.")
                    # Ensure we don't try to load a missing file repeatedly
                    self.active_mode = target_mode 

            # Control Rate (50Hz)
            if step_counter % self.sim_decimation == 0:
                dof_pos = self.data.qpos.astype(np.float32)[-self.num_dofs:]
                dof_vel = self.data.qvel.astype(np.float32)[-self.num_dofs:]
                quat = self.data.sensor('orientation').data.astype(np.float32)
                ang_vel = self.data.sensor('angular-velocity').data.astype(np.float32)
                
                mimic_obs = self._get_mimic_obs(step_counter // self.sim_decimation)
                rpy = quatToEuler(quat)
                obs_dof_vel = dof_vel.copy()
                obs_dof_vel[[4, 5, 10, 11]] = 0.
                
                obs_prop = np.concatenate([
                    ang_vel * 0.25,
                    rpy[:2],
                    (dof_pos - self.default_dof_pos) * 1.0,
                    obs_dof_vel * 0.05,
                    self.last_action,
                ])
                
                obs_hist = np.array(self.proprio_history_buf).flatten()
                obs_buf = np.concatenate([mimic_obs, obs_prop, obs_hist])
                obs_tensor = torch.from_numpy(obs_buf).float().unsqueeze(0).to(self.device)
                
                with torch.no_grad():
                    raw_action = self.policy_jit(obs_tensor).cpu().numpy().squeeze()
                
                self.last_action = raw_action.copy()
                scaled_actions = np.clip(raw_action, -10., 10.) * self.action_scale
                pd_target = scaled_actions + self.default_dof_pos
                self.proprio_history_buf.append(obs_prop)

            # Physics Step (1000Hz)
            dof_pos_p = self.data.qpos.astype(np.float32)[-self.num_dofs:]
            dof_vel_p = self.data.qvel.astype(np.float32)[-self.num_dofs:]
            torque = (pd_target - dof_pos_p) * self.stiffness - dof_vel_p * self.damping
            self.data.ctrl = np.clip(torque, -self.torque_limits, self.torque_limits)
            
            mujoco.mj_step(self.model, self.data)
            
            step_counter += 1
            expected_time = step_counter * self.sim_dt
            actual_time = time.time() - start_time
            if actual_time < expected_time:
                time.sleep(max(0, expected_time - actual_time))
            
            if step_counter % self.sim_decimation == 0:
                self.viewer.render()

        self.receiver.running = False
        self.receiver.join()
        self.viewer.close()

if __name__ == "__main__":
    jit_policy_pth = "assets/pretrained_checkpoints/pretrained.pt"
    device = "cuda" if torch.cuda.is_available() else "cpu"
    # Starting state
    init_file = os.path.join("assets/motions", "walk_stand.pkl")
    env = HumanoidEnv(policy_path=jit_policy_pth, motion_path=init_file, device=device)
    env.run()

'''


