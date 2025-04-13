import math
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")

    # Move to initial pose
    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)

    # --- Gripper Rotation: 90 degrees about Z (Yaw) ---
    T = np.identity(4)
    T[0, 3] = 0  # x
    T[2, 3] = 0  # z

    # 90 degree yaw rotation
    yaw = - np.pi / 2
    Rz = np.array([
        [math.cos(yaw), -math.sin(yaw), 0],
        [math.sin(yaw),  math.cos(yaw), 0],
        [0,              0,             1]
    ])
    T[:3, :3] = Rz
    locobot.arm.set_ee_pose_matrix(T)

    # Continue normal sequence
    locobot.arm.set_single_joint_position("waist", math.pi/4.0)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.close()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.set_single_joint_position("waist", -math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(pitch=1.5)
    locobot.arm.set_ee_cartesian_trajectory(pitch=-1.5)
    locobot.arm.set_single_joint_position("waist", math.pi/4.0)
    locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.25)
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x=-0.1, z=0.25)
    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()