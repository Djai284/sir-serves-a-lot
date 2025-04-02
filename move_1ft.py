from interbotix_xs_modules.locobot import InterbotixLocobotXS

def main():
    locobot = InterbotixLocobotXS(
        robot_model="locobot_wx250s",
        arm_model="mobile_wx250s",
        # use_base=True
    )
    
    distance_in_meters = 0.3048  # 1 foot = 0.3048 meters
    locobot.base.move_forward(distance=distance_in_meters)

if __name__ == '__main__':
    main()
