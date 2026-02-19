import mujoco
import mujoco.viewer
import numpy as np
import glfw

# ======================================
# LOAD MODEL
# ======================================
model = mujoco.MjModel.from_xml_path("g1_29dof.xml")
data = mujoco.MjData(model)

print("Actuators:", model.nu)

# ======================================
# GAINS (LEGS VERY STIFF)
# ======================================
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
        # arms softer
        Kp[i] = 30; Kd[i] = 3

# ======================================
# TARGET POSE
# ======================================
q_target = np.zeros(model.nu)

def set_joint(name, value):
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    q_target[idx] = value

# Stable crouch
set_joint("left_hip_pitch_joint", -0.4)
set_joint("right_hip_pitch_joint", -0.4)
set_joint("left_knee_joint", 0.8)
set_joint("right_knee_joint", 0.8)
set_joint("left_ankle_pitch_joint", -0.4)
set_joint("right_ankle_pitch_joint", -0.4)

set_joint("waist_pitch_joint", 0.0)
set_joint("waist_roll_joint", 0.0)
set_joint("waist_yaw_joint", 0.0)

set_joint("left_shoulder_pitch_joint", 0.3)
set_joint("right_shoulder_pitch_joint", 0.3)

# Arm actuator indices
left_arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                             "left_shoulder_pitch_joint")
right_arm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              "right_shoulder_pitch_joint")

# ======================================
# SIMULATION LOOP
# ======================================
with mujoco.viewer.launch_passive(model, data) as viewer:

    # Get GLFW window
    window = glfw.get_current_context()

    while viewer.is_running():

        # ---- READ ARROW KEYS ----
        step = 0.02

        if glfw.get_key(window, glfw.KEY_UP) == glfw.PRESS:
            q_target[left_arm] += step

        if glfw.get_key(window, glfw.KEY_DOWN) == glfw.PRESS:
            q_target[left_arm] -= step

        if glfw.get_key(window, glfw.KEY_RIGHT) == glfw.PRESS:
            q_target[right_arm] += step

        if glfw.get_key(window, glfw.KEY_LEFT) == glfw.PRESS:
            q_target[right_arm] -= step

        # ---- PD CONTROL ----
        q = data.qpos[7:]
        qdot = data.qvel[6:]

        tau = Kp * (q_target - q) - Kd * qdot

        for i in range(model.nu):
            frcrange = model.actuator_forcerange[i]
            tau[i] = np.clip(tau[i], frcrange[0], frcrange[1])

        data.ctrl[:] = tau

        mujoco.mj_step(model, data)
        viewer.sync()
