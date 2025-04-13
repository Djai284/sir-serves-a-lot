import rospy
import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class ArmController:
  def __init__(self, robot_model="locobot_wx250s", arm_model="mobile_wx250s"):
    self.robot_model = robot_model
    self.arm_model = arm_model
    self.locobot = None
    
    rospy.loginfo("Arm controller initialized for %s", self.robot_model)
  
  def connect(self, locobot):
    self.locobot = locobot
    
  def initialize(self):
    if self.locobot is None:
      self.locobot = InterbotixLocobotXS(
        robot_model=self.robot_model, 
        arm_model=self.arm_model
      )
    
    self.home_position()
  
  def home_position(self):
    self.locobot.arm.go_to_home_pose()
    rospy.loginfo("Arm moved to home position")
  
  def sleep_position(self):
    self.locobot.arm.go_to_sleep_pose()
    rospy.loginfo("Arm moved to sleep position")
  
  def prepare_for_grab(self):
    self.locobot.arm.set_ee_pose_components(x=0.3, z=0.2)
    self.locobot.gripper.open()
    rospy.loginfo("Arm positioned for grab")
    return True
  
  def grab_object(self):
    self.locobot.arm.set_ee_cartesian_trajectory(x=0.1, z=-0.2)
    rospy.sleep(1)
    
    self.locobot.gripper.close()
    rospy.sleep(1)
    
    self.locobot.arm.set_ee_cartesian_trajectory(z=0.15)
    
    rospy.loginfo("Object grabbed")
    return True
  
  def place_object(self, position="right"):
    if position == "right":
      self.locobot.arm.set_single_joint_position("waist", -math.pi/4.0)
    elif position == "left":
      self.locobot.arm.set_single_joint_position("waist", math.pi/4.0)
    elif position == "center":
      self.locobot.arm.set_single_joint_position("waist", 0)
        
    self.locobot.arm.set_ee_cartesian_trajectory(z=-0.15)
    rospy.sleep(1)
    
    self.locobot.gripper.open()
    rospy.sleep(1)
    
    self.locobot.arm.set_ee_cartesian_trajectory(z=0.15)
    
    self.home_position()
    
    rospy.loginfo("Object placed at %s position", position)
    return True
