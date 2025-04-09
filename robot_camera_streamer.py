# robot_camera_streamer.py
import socket
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time

HOST = '172.20.10.3'  # change this
PORT = 9999

bridge = CvBridge()
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

latest_frame = None
lock = threading.Lock()

def image_callback(msg):
    global latest_frame
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(e)
        return

    with lock:
        latest_frame = frame

def sender_loop():
    rate = rospy.Rate(10)  # Send at ~10 FPS
    while not rospy.is_shutdown():
        with lock:
            frame = latest_frame.copy() if latest_frame is not None else None

        if frame is None:
            rate.sleep()
            continue

        _, jpeg = cv2.imencode('.jpg', frame)
        data = jpeg.tobytes()
        size = len(data).to_bytes(4, byteorder='big')
        try:
            client_socket.sendall(size + data)
        except Exception as e:
            rospy.logerr(e)
            break

        rate.sleep()

def main():
    rospy.init_node('camera_streamer')
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    thread = threading.Thread(target=sender_loop)
    thread.start()

    rospy.spin()


if __name__ == '__main__':
    main()
