# manual mode only

import mujoco
import mujoco.viewer
import numpy as np
import socket
import struct
import threading
import time

# ==============================
# Setup Model & Data
# ==============================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

# ==============================
# PD Gains 
# ==============================
Kp = np.ones(model.nu) * 400
Kd = np.ones(model.nu) * 25

  
for i in range(model.nu):
    name = model.actuator(i).name
    
    
    if "shoulder" in name:
        Kp[i] = 60.0   
        Kd[i] = 3.0         
    
    elif "elbow" in name:
        Kp[i] = 30.0   
        Kd[i] = 1.5          
    
    elif "wrist" in name:
        Kp[i] = 10.0
        Kd[i] = 0.5      
    
    elif "hip" in name or "knee" in name:
        Kp[i] = 400.0 
        Kd[i] = 10.0    


# ==============================
# UDP Network Setup
# ==============================
q_target = np.zeros(model.nu)
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def network_loop():
    global q_target
    while True:
        try:
            data_packet, addr = sock.recvfrom(1024)
            if data_packet:
                unpacked = struct.unpack('f' * model.nu, data_packet)
                q_target[:] = np.array(unpacked)
                # Add this line:
                print(f"Received from Unity: {q_target[0]}") 
        except:
            continue

# Start network listener in a background thread
threading.Thread(target=network_loop, daemon=True).start()


# ==============================
# Main Simulation Loop
# ==============================
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # PD Control Loop
        tau = np.zeros(model.nu)
        for i in range(model.nu):
            joint_id = model.actuator_trnid[i][0]
            q = data.qpos[model.jnt_qposadr[joint_id]]
            qdot = data.qvel[model.jnt_dofadr[joint_id]]
            
            # Apply your PD math
            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        viewer.sync()

'''  


#toggle switching

import mujoco
import mujoco.viewer
import numpy as np
import socket
import struct
import threading
import torch
from collections import deque

# ==============================
# Setup Model & Data
# ==============================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

# ==============================
# AI Policy Setup (G1 Walking)
# ==============================
DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
POLICY = torch.jit.load("assets/pretrained_checkpoints/pretrained.pt", map_location=DEVICE)
HISTORY_LEN = 20
# Observation size for G1 Proprioception: ang_vel(3) + rpy(2) + dof_pos(23) + dof_vel(23) + last_action(23) = 74
proprio_history = deque([np.zeros(74) for _ in range(HISTORY_LEN)], maxlen=HISTORY_LEN)

# Default pose for the 23 joints the policy expects
DEFAULT_DOF_POS = np.array([-0.2, 0, 0, 0.4, -0.2, 0, -0.2, 0, 0, 0.4, -0.2, 0, 0, 0, 0, 0, 0.4, 0, 1.2, 0, -0.4, 0, 1.2])

# Indices of the 23 joints in your 29-DOF model (Skipping wrists/hands)
# This mapping assumes your 29-DOF XML order matches the policy's 23-joint list
policy_joint_indices = np.arange(23) 

# ==============================
# Gain Profiles
# ==============================
# Profile 0: Manual (Stiff)
Kp_manual = np.array([400.0 if "hip" in model.actuator(i).name or "knee" in model.actuator(i).name else 60.0 for i in range(model.nu)])
Kd_manual = np.array([10.0 if "hip" in model.actuator(i).name or "knee" in model.actuator(i).name else 3.0 for i in range(model.nu)])

# Profile 1: AI Walking (Softer)
Kp_ai = np.ones(model.nu) * 100.0
Kd_ai = np.ones(model.nu) * 5.0

# Current active gains
Kp = Kp_manual.copy()
Kd = Kd_manual.copy()

# ==============================
# UDP Network Setup
# ==============================
q_target = np.zeros(model.nu)
current_mode = 0.0 # 0=Manual, 1=Walk
last_action = np.zeros(23)

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)

def network_loop():
    global q_target, current_mode, Kp, Kd
    while True:
        try:
            data_packet, addr = sock.recvfrom(1024)
            if data_packet:
                # Unpack 30 floats (29 joints + 1 mode)
                unpacked = struct.unpack('f' * 30, data_packet)
                incoming_mode = unpacked[29]
                
                # Update Gains if mode changed
                if incoming_mode != current_mode:
                    if incoming_mode == 1.0:
                        Kp, Kd = Kp_ai, Kd_ai
                        model.dof_damping[:] = 2.0 # Relax physics
                    else:
                        Kp, Kd = Kp_manual, Kd_manual
                        model.dof_damping[:] = 8.0 # Stiffen physics
                    current_mode = incoming_mode

                # Update manual target if in manual mode
                if current_mode == 0.0:
                    q_target[:] = np.array(unpacked[:29])
        except:
            continue

threading.Thread(target=network_loop, daemon=True).start()

# ==============================
# Main Simulation Loop
# ==============================
with mujoco.viewer.launch_passive(model, data) as viewer:
    step_count = 0
    while viewer.is_running():
        step_start = time.time()

        if current_mode == 1.0:
            # AI Inference every 20ms (Sim Decimation 20)
            if step_count % 20 == 0:
                # 1. Extract State
                q_physical = data.qpos[-29:][policy_joint_indices]
                v_physical = data.qvel[-29:][policy_joint_indices]
                ang_vel = data.sensor('angular-velocity').data * 0.25
                
                # 2. Build Proprioception Observation
                # (Simple version: adapt based on your specific policy's obs space)
                obs_prop = np.concatenate([
                    ang_vel, [0, 0], # ang_vel + rpy placeholder
                    (q_physical - DEFAULT_DOF_POS),
                    v_physical * 0.05,
                    last_action
                ])
                
                # 3. Handle History
                proprio_history.append(obs_prop)
                obs_hist = np.array(proprio_history).flatten()
                
                # 4. Infer
                obs_tensor = torch.from_numpy(obs_hist).float().unsqueeze(0).to(DEVICE)
                with torch.no_grad():
                    action = POLICY(obs_tensor).cpu().numpy().squeeze()
                
                last_action = action
                # Map action to targets (indices 0-22)
                q_target[policy_joint_indices] = (action * 0.5) + DEFAULT_DOF_POS
                # Keep wrists/hands (23-28) at 0 during walk
                q_target[23:29] = 0.0

        # Standard PD Control Loop
        tau = np.zeros(model.nu)
        for i in range(model.nu):
            joint_id = model.actuator_trnid[i][0]
            q = data.qpos[model.jnt_qposadr[joint_id]]
            qdot = data.qvel[model.jnt_dofadr[joint_id]]
            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

        data.ctrl[:] = tau
        mujoco.mj_step(model, data)
        
        if step_count % 10 == 0:
            viewer.sync()
        
        step_count += 1


'''