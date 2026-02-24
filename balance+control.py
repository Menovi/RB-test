'''

import mujoco
import mujoco.viewer
import numpy as np
import threading
import tkinter as tk

# =====================================
# Load model
# =====================================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

print("Actuators:", model.nu)

# =====================================
# EXACT ORIGINAL STIFF GAINS
# =====================================
Kp = np.zeros(model.nu)
Kd = np.zeros(model.nu)

for i in range(model.nu):
    name = model.actuator(i).name

    if "hip_pitch" in name:
        Kp[i] = 400; Kd[i] = 25
    elif "hip_roll" in name:
        Kp[i] = 350; Kd[i] = 20
    elif "hip_yaw" in name:
        Kp[i] = 200; Kd[i] = 10
    elif "knee" in name:
        Kp[i] = 700; Kd[i] = 40
    elif "ankle_pitch" in name:
        Kp[i] = 300; Kd[i] = 20
    elif "ankle_roll" in name:
        Kp[i] = 250; Kd[i] = 15
    elif "waist" in name:
        Kp[i] = 200; Kd[i] = 15
    else:
        # Arms still controlled but weaker
        Kp[i] = 400; Kd[i] = 25

# =====================================
# STABLE STATUE POSE
# =====================================
q_target = np.zeros(model.nu)

def set_joint(name, value):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    q_target[idx] = value

set_joint("left_hip_pitch_joint", -0.4)
set_joint("right_hip_pitch_joint", -0.4)
set_joint("left_knee_joint", 0.8)
set_joint("right_knee_joint", 0.8)
set_joint("left_ankle_pitch_joint", -0.4)
set_joint("right_ankle_pitch_joint", -0.4)
set_joint("left_hip_roll_joint", 0.0)
set_joint("right_hip_roll_joint", 0.0)
set_joint("waist_pitch_joint", 0.0)
set_joint("waist_roll_joint", 0.0)
set_joint("waist_yaw_joint", 0.0)
set_joint("left_shoulder_pitch_joint", 0.3)
set_joint("right_shoulder_pitch_joint", 0.3)

# Arm actuator indices
left_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "left_shoulder_pitch_joint")

right_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "right_shoulder_pitch_joint")

# =====================================
# BUTTON CONTROL WINDOW
# =====================================
def gui():
    root = tk.Tk()
    root.title("Arm Control")

    def l_up():    q_target[left_arm] += 0.1
    def l_down():  q_target[left_arm] -= 0.1
    def r_up():    q_target[right_arm] += 0.1
    def r_down():  q_target[right_arm] -= 0.1

    tk.Button(root, text="Left Arm Up", command=l_up).pack()
    tk.Button(root, text="Left Arm Down", command=l_down).pack()
    tk.Button(root, text="Right Arm Up", command=r_up).pack()
    tk.Button(root, text="Right Arm Down", command=r_down).pack()

    root.mainloop()

threading.Thread(target=gui, daemon=True).start()

# =====================================
# SIM LOOP (UNCHANGED PHYSICS)
# =====================================
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        tau = np.zeros(model.nu)

        for i in range(model.nu):

            joint_id = model.actuator_trnid[i][0]   # joint attached to actuator
            qpos_adr = model.jnt_qposadr[joint_id]
            qvel_adr = model.jnt_dofadr[joint_id]

            q = data.qpos[qpos_adr]
            qdot = data.qvel[qvel_adr]

            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

            frcrange = model.actuator_forcerange[i]
            tau[i] = np.clip(tau[i], frcrange[0], frcrange[1])

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()
        
print(left_arm)
print(model.actuator_forcerange[left_arm])     









#version 2 with control window
import mujoco
import mujoco.viewer
import numpy as np
import threading
import tkinter as tk

# ==============================
# Load model
# ==============================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

# ==============================
# Gains
# ==============================
Kp = np.ones(model.nu) * 400
Kd = np.ones(model.nu) * 25

for i in range(model.nu):
    name = model.actuator(i).name
    if "shoulder_pitch" in name:
        Kp[i] = 2000
        Kd[i] = 150

# ==============================
# Targets
# ==============================
q_target = np.zeros(model.nu)

def set_joint(name, value):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    q_target[idx] = value

# Standing pose
set_joint("left_hip_pitch_joint", -0.4)
set_joint("right_hip_pitch_joint", -0.4)
set_joint("left_knee_joint", 0.8)
set_joint("right_knee_joint", 0.8)
set_joint("left_ankle_pitch_joint", -0.4)
set_joint("right_ankle_pitch_joint", -0.4)

set_joint("left_shoulder_pitch_joint", 0.3)
set_joint("right_shoulder_pitch_joint", 0.3)

left_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "left_shoulder_pitch_joint")

right_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "right_shoulder_pitch_joint")

# ==============================
# GUI
# ==============================
def gui():
    root = tk.Tk()
    root.title("Arm Control")

    def l_up():
        q_target[left_arm] += 0.5
        #print("Left:", q_target[left_arm])

    def l_down():
        q_target[left_arm] -= 0.5

    def r_up():
        q_target[right_arm] += 0.5

    def r_down():
        q_target[right_arm] -= 0.5

    tk.Button(root, text="Left Down", command=l_up).pack()
    tk.Button(root, text="Left Up", command=l_down).pack()
    tk.Button(root, text="Right Down", command=r_up).pack()
    tk.Button(root, text="Right Up", command=r_down).pack()

    root.mainloop()

threading.Thread(target=gui, daemon=True).start()

# ==============================
# Simulation
# ==============================
with mujoco.viewer.launch_passive(
        model, data,
        show_left_ui=True,
        show_right_ui=True) as viewer:

    while viewer.is_running():

        tau = np.zeros(model.nu)

        for i in range(model.nu):
            joint_id = model.actuator_trnid[i][0]
            qpos_adr = model.jnt_qposadr[joint_id]
            qvel_adr = model.jnt_dofadr[joint_id]

            q = data.qpos[qpos_adr]
            qdot = data.qvel[qvel_adr]

            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()
        



#version 3 with improved controls

import mujoco
import mujoco.viewer
import numpy as np
import threading
import tkinter as tk
from mujoco.glfw import glfw

# ==============================
# Load model
# ==============================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

# ==============================
# Gains
# ==============================
Kp = np.ones(model.nu) * 400
Kd = np.ones(model.nu) * 25

for i in range(model.nu):
    name = model.actuator(i).name
    if "shoulder_pitch" in name:
        Kp[i] = 2000
        Kd[i] = 150

# ==============================
# Targets
# ==============================
q_target = np.zeros(model.nu)

def set_joint(name, value):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    q_target[idx] = value

# Standing pose
set_joint("left_hip_pitch_joint", -0.4)
set_joint("right_hip_pitch_joint", -0.4)
set_joint("left_knee_joint", 0.8)
set_joint("right_knee_joint", 0.8)
set_joint("left_ankle_pitch_joint", -0.4)
set_joint("right_ankle_pitch_joint", -0.4)

set_joint("left_shoulder_pitch_joint", 0.3)
set_joint("right_shoulder_pitch_joint", 0.3)

left_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "left_shoulder_pitch_joint")

right_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "right_shoulder_pitch_joint")

# ==============================
# TK BUTTON GUI (unchanged)
# ==============================
def gui():
    root = tk.Tk()
    root.title("Arm Control")

    def l_up():
        q_target[left_arm] += 0.5

    def l_down():
        q_target[left_arm] -= 0.5

    def r_up():
        q_target[right_arm] += 0.5

    def r_down():
        q_target[right_arm] -= 0.5

    tk.Button(root, text="Left Up", command=l_up).pack()
    tk.Button(root, text="Left Down", command=l_down).pack()
    tk.Button(root, text="Right Up", command=r_up).pack()
    tk.Button(root, text="Right Down", command=r_down).pack()

    root.mainloop()

threading.Thread(target=gui, daemon=True).start()

# ==============================
# Arrow Key State Tracking
# ==============================
from mujoco.glfw import glfw
import time

# ==============================
# Key state storage
# ==============================
key_last_pressed = {
    "up": 0.0,
    "down": 0.0,
    "left": 0.0,
    "right": 0.0,
}

HOLD_TIMEOUT = 0.15 

def key_callback(key):
    now = time.time()

    if key == glfw.KEY_UP:
        key_last_pressed["up"] = now
    elif key == glfw.KEY_DOWN:
        key_last_pressed["down"] = now
    elif key == glfw.KEY_LEFT:
        key_last_pressed["left"] = now
    elif key == glfw.KEY_RIGHT:
        key_last_pressed["right"] = now


# ==============================
# Simulation
# ==============================
with mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=True,
        show_right_ui=True,
        key_callback=key_callback) as viewer:

    move_speed = 20.0  # radians p s

    while viewer.is_running():

        dt = model.opt.timestep
        now = time.time()

        
        if now - key_last_pressed["up"] < HOLD_TIMEOUT:
            q_target[left_arm] += move_speed * dt

        if now - key_last_pressed["down"] < HOLD_TIMEOUT:
            q_target[left_arm] -= move_speed * dt

        if now - key_last_pressed["right"] < HOLD_TIMEOUT:
            q_target[right_arm] += move_speed * dt

        if now - key_last_pressed["left"] < HOLD_TIMEOUT:
            q_target[right_arm] -= move_speed * dt

        tau = np.zeros(model.nu)

        for i in range(model.nu):
            joint_id = model.actuator_trnid[i][0]
            qpos_adr = model.jnt_qposadr[joint_id]
            qvel_adr = model.jnt_dofadr[joint_id]

            q = data.qpos[qpos_adr]
            qdot = data.qvel[qvel_adr]

            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()
        
        
'''
        
#version 4 fix delay control   



import mujoco
import mujoco.viewer
import numpy as np
import threading
import tkinter as tk
from mujoco.glfw import glfw

# ==============================
# Load model
# ==============================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

# ==============================
# Gains
# ==============================
Kp = np.ones(model.nu) * 400
Kd = np.ones(model.nu) * 25

for i in range(model.nu):
    name = model.actuator(i).name
    if "shoulder_pitch" in name:
        Kp[i] = 2000
        Kd[i] = 150

# ==============================
# Targets
# ==============================
q_target = np.zeros(model.nu)

def set_joint(name, value):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    q_target[idx] = value

# Standing pose
set_joint("left_hip_pitch_joint", -0.4)
set_joint("right_hip_pitch_joint", -0.4)
set_joint("left_knee_joint", 0.8)
set_joint("right_knee_joint", 0.8)
set_joint("left_ankle_pitch_joint", -0.4)
set_joint("right_ankle_pitch_joint", -0.4)

set_joint("left_shoulder_pitch_joint", 0.3)
set_joint("right_shoulder_pitch_joint", 0.3)

left_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "left_shoulder_pitch_joint")

right_arm = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "right_shoulder_pitch_joint")

# ==============================
# TK BUTTON GUI (unchanged)
# ==============================
def gui():
    root = tk.Tk()
    root.title("Arm Control")

    def l_up():
        q_target[left_arm] += 0.5

    def l_down():
        q_target[left_arm] -= 0.5

    def r_up():
        q_target[right_arm] += 0.5

    def r_down():
        q_target[right_arm] -= 0.5

    tk.Button(root, text="Left Up", command=l_up).pack()
    tk.Button(root, text="Left Down", command=l_down).pack()
    tk.Button(root, text="Right Up", command=r_up).pack()
    tk.Button(root, text="Right Down", command=r_down).pack()

    root.mainloop()

threading.Thread(target=gui, daemon=True).start()

from mujoco.glfw import glfw

move_speed = 0.5  # radians per key repeat event

def key_callback(key):

    if key == glfw.KEY_UP:
        q_target[left_arm] += move_speed

    elif key == glfw.KEY_DOWN:
        q_target[left_arm] -= move_speed

    elif key == glfw.KEY_RIGHT:
        q_target[right_arm] += move_speed

    elif key == glfw.KEY_LEFT:
        q_target[right_arm] -= move_speed


with mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=True,
        show_right_ui=True,
        key_callback=key_callback) as viewer:

    step_counter = 0

    while viewer.is_running():

        tau = np.zeros(model.nu)

        for i in range(model.nu):
            joint_id = model.actuator_trnid[i][0]
            qpos_adr = model.jnt_qposadr[joint_id]
            qvel_adr = model.jnt_dofadr[joint_id]

            q = data.qpos[qpos_adr]
            qdot = data.qvel[qvel_adr]

            tau[i] = Kp[i] * (q_target[i] - q) - Kd[i] * qdot

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()

        # Print joint info every 200 steps
        step_counter += 1
        if step_counter % 200 == 0:

            left_joint_id = model.actuator_trnid[left_arm][0]
            right_joint_id = model.actuator_trnid[right_arm][0]

            left_q = data.qpos[model.jnt_qposadr[left_joint_id]]
            right_q = data.qpos[model.jnt_qposadr[right_joint_id]]

            print(f"\nLEFT ARM")
            print(f"  Target: {q_target[left_arm]:.3f}")
            print(f"  Actual: {left_q:.3f}")
            print(f"  Torque: {data.ctrl[left_arm]:.3f}")

            print(f"RIGHT ARM")
            print(f"  Target: {q_target[right_arm]:.3f}")
            print(f"  Actual: {right_q:.3f}")
            print(f"  Torque: {data.ctrl[right_arm]:.3f}")