#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time

class YOLOViewer:
    def __init__(self):
        rospy.init_node("yolo_camera_debug", anonymous=True)

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")  # You can replace with your custom model
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
        rospy.loginfo("Subscribed to /camera/color/image_raw")

    def callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Could not convert image: %s", e)
            return

        start_time = time.time()
        results = self.model(frame)[0]

        for box in results.boxes:
            conf = float(box.conf[0])
            if conf < 0.4:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = f"{self.model.names[cls_id]} {conf:.2f}"

            # Draw bounding box and label
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        fps = 1.0 / (time.time() - start_time)
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # Display the image
        cv2.imshow("YOLOv8 Detection", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            rospy.signal_shutdown("User exited.")

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = YOLOViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
