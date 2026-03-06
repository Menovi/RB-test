
#approach 1 ghost toggle
#23dof is a ghost and 29dof tries to mimic it

import mujoco
import mujoco.viewer
import numpy as np
import socket
import struct
import threading
import torch
import time
from collections import deque
from utils.motion_lib import MotionLib

# ==============================
# Helper Functions from sim2sim
# ==============================
@torch.jit.script
def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]; q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

def euler_from_quaternion(quat_angle):
    x, y, z, w = quat_angle[:,0], quat_angle[:,1], quat_angle[:,2], quat_angle[:,3]
    roll = torch.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = torch.asin(torch.clip(2.0 * (w * y - z * x), -1, 1))
    yaw = torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw

def quatToEuler(quat):
    qw, qx, qy, qz = quat
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    sinp = 2 * (qw * qy - qz * qx)
    pitch = np.where(np.abs(sinp) >= 1, np.copysign(np.pi / 2, sinp), np.arcsin(sinp))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return np.array([roll, pitch, yaw])

# ==============================
# 1. SETUP BOTH MODELS
# ==============================
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"

# Load Manual Model (The one Unity sees)
m_manual = mujoco.MjModel.from_xml_path("g1_29dof.xml")
d_manual = mujoco.MjData(m_manual)

# Load AI Model (The one the Policy expects)
m_ai = mujoco.MjModel.from_xml_path("assets/robots/g1/g1.xml")
d_ai = mujoco.MjData(m_ai)

# Pretrained Parameters (Exactly from sim2sim)
STIFFNESS_AI = np.array([100,100,100,150,40,40, 100,100,100,150,40,40, 150,150,150, 40,40,40,40, 40,40,40,40])
DAMPING_AI = np.array([2,2,2,4,2,2, 2,2,2,4,2,2, 4,4,4, 5,5,5,5, 5,5,5,5])
DEFAULT_DOF_POS_AI = np.array([-0.2, 0.0, 0.0, 0.4, -0.2, 0.0, -0.2, 0.0, 0.0, 0.4, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4, 0.0, 1.2, 0.0, -0.4, 0.0, 1.2])
TAR_OBS_STEPS = torch.tensor([1, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95], device=DEVICE)

# Manual PD Gains (Original 29-DOF values)
Kp_manual = np.ones(m_manual.nu) * 400
Kd_manual = np.ones(m_manual.nu) * 25
for i in range(m_manual.nu):
    name = m_manual.actuator(i).name
    if "shoulder" in name: Kp_manual[i], Kd_manual[i] = 60.0, 3.0
    elif "elbow" in name: Kp_manual[i], Kd_manual[i] = 30.0, 1.5
    elif "wrist" in name: Kp_manual[i], Kd_manual[i] = 10.0, 0.5
    elif "hip" in name or "knee" in name: Kp_manual[i], Kd_manual[i] = 400.0, 10.0

# State Variables
q_target_unity = np.zeros(29)
current_mode = 0.0
last_action = np.zeros(23, dtype=np.float32)
proprio_history = deque([np.zeros(74) for _ in range(20)], maxlen=20)
walk_start_time = 0

# Load AI Assets
policy_jit = torch.jit.load("assets/pretrained_checkpoints/pretrained.pt", map_location=DEVICE)
motion_lib = MotionLib("assets/motions/walk_stand.pkl", DEVICE)

# ==============================
# 2. NETWORKING
# ==============================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 5005))
sock.setblocking(False)

def network_loop():
    global q_target_unity, current_mode, walk_start_time
    while True:
        try:
            data_packet, _ = sock.recvfrom(1024)
            unpacked = struct.unpack('f' * 30, data_packet)
            new_mode = unpacked[29]
            if new_mode == 1.0 and current_mode == 0.0:
                # SYNC AI MODEL TO MANUAL MODEL STATE UPON START
                d_ai.qpos[:7] = d_manual.qpos[:7] # Base pos/quat
                d_ai.qpos[7:30] = d_manual.qpos[7:30] # Joints
                walk_start_time = time.time()
            current_mode = new_mode
            q_target_unity[:] = np.array(unpacked[:29])
        except: continue

threading.Thread(target=network_loop, daemon=True).start()

def get_mimic_obs(curr_step):
    control_dt = 0.02
    motion_times = torch.tensor([curr_step * control_dt], device=DEVICE).unsqueeze(-1)
    obs_motion_times = (TAR_OBS_STEPS * control_dt + motion_times).flatten()
    motion_ids = torch.zeros(len(TAR_OBS_STEPS), dtype=torch.int, device=DEVICE)
    root_pos, root_rot, root_vel, root_ang_vel, dof_pos, _ = motion_lib.calc_motion_frame(motion_ids, obs_motion_times)
    r, p, _ = euler_from_quaternion(root_rot)
    rv = quat_rotate_inverse(root_rot, root_vel)
    rav = quat_rotate_inverse(root_rot, root_ang_vel)
    mimic = torch.cat((root_pos[..., 2:3], r.unsqueeze(-1), p.unsqueeze(-1), rv, rav[..., 2:3], dof_pos), dim=-1)
    return mimic.reshape(1, -1).cpu().numpy().squeeze()

# ==============================
# 3. MAIN LOOP
# ==============================
with mujoco.viewer.launch_passive(m_manual, d_manual) as viewer:
    step_count = 0
    pd_target_ai = np.zeros(23)
    
    while viewer.is_running():
        # --- AI BLOCK (Every 20ms) ---
        if step_count % 20 == 0:
            dof_pos_23 = d_ai.qpos[7:30]
            dof_vel_23 = d_ai.qvel[6:29]
            quat = d_ai.sensor('orientation').data
            ang_vel = d_ai.sensor('angular-velocity').data
            rpy = quatToEuler(quat)
            
            obs_dof_vel = dof_vel_23.copy()
            obs_dof_vel[[4, 5, 10, 11]] = 0. 
            obs_prop = np.concatenate([ang_vel * 0.25, rpy[:2], (dof_pos_23 - DEFAULT_DOF_POS_AI), obs_dof_vel * 0.05, last_action])
            
            if current_mode == 1.0:
                curr_ts = int((time.time() - walk_start_time) / 0.02)
                mimic_obs = get_mimic_obs(curr_ts)
                obs_buf = np.concatenate([mimic_obs, obs_prop, np.array(proprio_history).flatten()])
                with torch.no_grad():
                    raw_action = policy_jit(torch.from_numpy(obs_buf).float().unsqueeze(0).to(DEVICE)).cpu().numpy().squeeze()
                last_action = raw_action.copy()
                pd_target_ai = (np.clip(raw_action, -10, 10) * 0.5) + DEFAULT_DOF_POS_AI
            
            proprio_history.append(obs_prop)

        # --- PHYSICS & CONTROL BLOCK ---
        if current_mode == 1.0:
            # 1. Step the AI Ghost
            tau_ai = STIFFNESS_AI * (pd_target_ai - d_ai.qpos[7:30]) - DAMPING_AI * d_ai.qvel[6:29]
            d_ai.ctrl[:] = np.clip(tau_ai, -80, 80)
            mujoco.mj_step(m_ai, d_ai)
            
            # 2. Sync ONLY the 23 joints the AI knows
            d_manual.qpos[:30] = d_ai.qpos[:30]
            d_manual.qvel[:29] = d_ai.qvel[:29]
            
            # 3. LOCK THE WRISTS (Indices 23-28)
            # We use high damping on the manual actuators to keep them frozen
            for i in range(23, 29):
                joint_id = m_manual.actuator_trnid[i][0]
                q = d_manual.qpos[m_manual.jnt_qposadr[joint_id]]
                qdot = d_manual.qvel[m_manual.jnt_dofadr[joint_id]]
                
                # Target is 0 (neutral pose), with high stiffness to prevent flopping
                d_manual.ctrl[i] = 100 * (0.0 - q) - 10.0 * qdot 
        else:
            # Standard Manual Control
            tau = np.zeros(m_manual.nu)
            for i in range(m_manual.nu):
                joint_id = m_manual.actuator_trnid[i][0]
                q, qdot = d_manual.qpos[m_manual.jnt_qposadr[joint_id]], d_manual.qvel[m_manual.jnt_dofadr[joint_id]]
                tau[i] = Kp_manual[i] * (q_target_unity[i] - q) - Kd_manual[i] * qdot
            d_manual.ctrl[:] = tau
            
        mujoco.mj_step(m_manual, d_manual)
        
        if step_count % 20 == 0: viewer.sync()
        step_count += 1



#approach 2 fixing bugs of 1

#idea: despawn 29dof when toggled
#spawn 23dof and walk then despawn when toggled (also spawn back 29dof)

#also test with the 29dof walking if this doesnt work out


