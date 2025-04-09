import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraViewer:
    def __init__(self):
        rospy.init_node('camera_viewer', anonymous=True)

        # Adjust this topic name to match your robot's camera topic
        self.image_topic = "/camera/color/image_raw"

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.callback)
        rospy.loginfo("Subscribed to camera feed: %s", self.image_topic)

    def callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

    def run(self):
        rospy.spin()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        viewer = CameraViewer()
        viewer.run()
    except rospy.ROSInterruptException:
        pass
