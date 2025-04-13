from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np

def main():
    # Create a 4x4 identity matrix as the starting transformation.
    T_sd = np.identity(4)

    # Set the desired translation (position) of the end-effector.
    T_sd[0, 3] = 0  # x position
    T_sd[1, 3] = 0    # y position
    T_sd[2, 3] = 0  # z position

    # Define a 90 degree rotation (in radians) about the z-axis.
    theta = np.pi / 2  # 90 degrees in radians

    # Construct the rotation matrix for a 90° rotation about the z-axis.
    Rz = np.array([
        [np.cos(theta), -np.sin(theta), 0],
        [np.sin(theta),  np.cos(theta), 0],
        [0,              0,             1]
    ])

    # Replace the upper-left 3x3 rotation component of T_sd with Rz.
    T_sd[:3, :3] = Rz

    # Create the manipulator instance.
    bot = InterbotixManipulatorXS("wx250s", "arm", "gripper")

    # Home the arm, set the new pose, then return the arm to home and sleep poses.
    bot.arm.go_to_home_pose()
    bot.arm.set_ee_pose_matrix(T_sd)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()