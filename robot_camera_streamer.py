# robot_camera_streamer.py
import socket
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HOST = 'your_macbook_ip'  # change this
PORT = 9999

bridge = CvBridge()
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    _, jpeg = cv2.imencode('.jpg', frame)
    data = jpeg.tobytes()
    size = len(data).to_bytes(4, byteorder='big')
    try:
        client_socket.sendall(size + data)
    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('camera_streamer')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
