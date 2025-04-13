import math
import numpy as np
from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    locobot = InterbotixLocobotXS(robot_model="locobot_wx250s", arm_model="mobile_wx250s")

    # Move to initial pose
    locobot.arm.set_ee_pose_components(x=0.3, z=0.2)

    print(locobot.arm.group_info.num_joints)

    locobot.arm.go_to_home_pose()
    locobot.arm.go_to_sleep_pose()

if __name__ == '__main__':
    main()