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
# EXACT ORIGINAL STABLE GAINS (UNCHANGED)
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
        Kp[i] = 30; Kd[i] = 3

# =====================================
# EXACT STABLE POSE (UNCHANGED)
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

# Waist yaw actuator index
waist_yaw = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_ACTUATOR,
    "waist_yaw_joint")

# =====================================
# Control Window
# =====================================
def gui():
    root = tk.Tk()
    root.title("Waist Rotation Control")

    def rotate_left():
        q_target[waist_yaw] += 0.05  # small change

    def rotate_right():
        q_target[waist_yaw] -= 0.05

    tk.Button(root, text="Rotate Left", command=rotate_left).pack()
    tk.Button(root, text="Rotate Right", command=rotate_right).pack()

    root.mainloop()

threading.Thread(target=gui, daemon=True).start()

# =====================================
# SIM LOOP (UNCHANGED CONTROLLER)
# =====================================
with mujoco.viewer.launch_passive(model, data) as viewer:

    while viewer.is_running():

        q = data.qpos[7:]
        qdot = data.qvel[6:]

        tau = Kp * (q_target - q) - Kd * qdot

        for i in range(model.nu):
            frcrange = model.actuator_forcerange[i]
            tau[i] = np.clip(tau[i], frcrange[0], frcrange[1])

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()
