import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class ObjectDetection:
  def __init__(self, camera_streamer=None, target_classes=None):
    self.bridge = CvBridge()
    self.model = YOLO("yolov8n.pt")
    self.camera_streamer = camera_streamer
    
    if target_classes:
      self.target_classes = target_classes
    else:
      self.target_classes = ["plate", "cup", "bowl", "bottle"]
      
    self.detected_objects = []
    
    self.image_sub = rospy.Subscriber(
      '/camera/color/image_raw', 
      Image, 
      self.image_callback  
    )
    
    rospy.loginfo("Object detector initialized with target classes: %s", str(self.target_classes))
    
  def image_callback(self, msg):
    try:
      frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
      rospy.loginfo("Could not convert image: %s", e)
      return  
      
    results = self.model(frame)[0]
    
    self.detected_objects = []
    for box in results.boxes:
      conf = float(box.conf[0])
      
      if conf < 0.4: 
        continue
      
      cls_id = int(box.cls[0])  
      label = self.model.names[cls_id]
      
      if label in self.target_classes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        width = x2 - x1
        height = y2 - y1 
        
        self.detected_objects.append({
          "label": label,
          "confidence": conf,
          "center": (center_x, center_y),
          "box": (x1, y1, x2, y2),
          "size": width * height
        })
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
    if self.camera_streamer:
      self.camera_streamer.update_frame(frame)
      
    cv2.imshow("Object detection", frame)
    cv2.waitKey(1)
        
  def get_best_target(self):
    """ 
    sorts the list and gets the best 
    target based on confidence level
    """
    if not self.detected_objects:
      return None
    
    sorted_targets = sorted(
      self.detected_objects,
      key=lambda x: x["confidence"],
      reverse=True
    )
    
    return sorted_targets[0]
    
  def return_all_objects(self):
    """ Returns all the detected objects """
    return self.detected_objects
