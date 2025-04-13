import rospy
import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS


class Navigation():
  def __init__(self, robot_model="locobot_wx250s", arm_model="mobile_wx250s"):
    self.robot_model = robot_model
    self.arm_model = arm_model
    self.locobot = None
    self.image_width = 640
    self.image_height = 480
    
    rospy.loginfo("Initialized navigation")
  
  def move_to_object(self, target_object, stop_threshold=100000):
    """Move toward the target object
    
    Args:
        target_object: Dict with object detection data
        stop_threshold: Size threshold to stop approaching
        
    Returns:
        bool: True if reached object, False otherwise
    """
    
    if not target_object:
      rospy.logwarn("No target object provided")
      return False
        
    object_center_x = target_object["center"][0]
    center_offset = object_center_x - (self.image_width // 2)
    
    box_size = (target_object["box"][2] - target_object["box"][0]) * \
              (target_object["box"][3] - target_object["box"][1])
    
    if box_size > stop_threshold:
      self.locobot.base.move(0, 0, 0)  
      rospy.loginfo("Reached object, stopping (box size: %d)", box_size)
      return True
        
    if abs(center_offset) > 50:  
      turn_speed = 0.2 * (center_offset / (self.image_width // 2))
      turn_speed = max(min(turn_speed, 0.3), -0.3)
      
      self.locobot.base.move(0, 0, turn_speed)
      rospy.loginfo("Turning to center object. Offset: %d", center_offset)
    else:
      self.locobot.base.move(0.1, 0, 0)
      rospy.loginfo("Moving forward toward object (box size: %d)", box_size)
        
    return False
    
  def stop(self):
    """Stop all movement"""
    self.locobot.base.move(0, 0, 0)
    
  def connect(self, locobot):
    self.locobot = locobot
    
  def initialize(self):
    if self.locobot is None:
      self.locobot = InterbotixLocobotXS(
        robot_model=self.robot_model, 
        arm_model=self.arm_model,
        use_move_base_action=True
    )
  