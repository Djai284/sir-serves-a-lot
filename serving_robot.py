import rospy
import time
from interbotix_xs_modules.locobot import InterbotixLocobotXS

from object_detection import ObjectDetection
from arm_controller import ArmController
from navigation import Navigation
from camera_streamer import CameraStreamer

class ServingRobot:
  def __init__(self):
    rospy.init_node("serving_robot", anonymous=True)
    
    self.locobot = InterbotixLocobotXS(
      robot_model="locobot_wx250s", 
      arm_model="mobile_wx250s",
      use_move_base_action=True
    )
    
    self.streamer = CameraStreamer(host='0.0.0.0', port=9999)
    self.streamer.start_streaming()
    
    self.detector = ObjectDetection(camera_streamer=self.streamer)
    
    self.arm = ArmController()
    self.arm.connect(self.locobot)
    
    self.nav = Navigation()
    self.nav.connect(self.locobot)
    
    self.current_state = "IDLE"
    self.current_target = None
    
    rospy.loginfo("Serving Robot initialized and ready!")
    
  def run(self):
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
      if self.current_state == "IDLE":
        target = self.detector.get_best_target()
        if target:
          self.current_target = target
          self.current_state = "NAVIGATE"
          rospy.loginfo("Found target: %s, navigating", target["label"])
        
      elif self.current_state == "NAVIGATE":
        reached = self.nav.move_to_object(self.current_target)
        if reached:
          self.current_state = "PREPARE_GRAB"
          rospy.loginfo("Reached target, preparing to grab")
        
      elif self.current_state == "PREPARE_GRAB":
        success = self.arm.prepare_for_grab()
        if success:
          self.current_state = "GRAB"
          rospy.loginfo("Arm positioned, grabbing object")
          rospy.sleep(1)
        
      elif self.current_state == "GRAB":
        success = self.arm.grab_object()
        if success:
          self.current_state = "PLACE"
          rospy.loginfo("Object grabbed, moving to placement")
          rospy.sleep(1)
        
      elif self.current_state == "PLACE":
        success = self.arm.place_object("right")
        if success:
          self.current_state = "IDLE"
          self.current_target = None
          rospy.loginfo("Object placed, returning to idle")
          rospy.sleep(2)
      
      rate.sleep()

if __name__ == "__main__":
  try:
    robot = ServingRobot()
    robot.run()
  except rospy.ROSInterruptException:
    pass
