import socket
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import threading
import time
import numpy as np

class CameraStreamer:
  def __init__(self, host='0.0.0.0', port=9999):
    self.bridge = CvBridge()
    self.host = host
    self.port = port
    self.latest_frame = None
    self.lock = threading.Lock()
    self.client_socket = None
    
    # Start server socket
    self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.server_socket.bind((self.host, self.port))
    self.server_socket.listen(1)
    
    rospy.loginfo(f"Camera streamer initialized on {self.host}:{self.port}")
    
    # Start accept thread
    self.accept_thread = threading.Thread(target=self.accept_connections)
    self.accept_thread.daemon = True
    self.accept_thread.start()
  
  def accept_connections(self):
    """Accept client connections in background thread"""
    while not rospy.is_shutdown():
      rospy.loginfo("Waiting for client connection...")
      try:
        conn, addr = self.server_socket.accept()
        rospy.loginfo(f"Client connected from {addr}")
        self.client_socket = conn
      except Exception as e:
        rospy.logerr(f"Connection error: {e}")
      rospy.sleep(1)

  def update_frame(self, frame):
    """Update the latest frame to be streamed"""
    with self.lock:
      self.latest_frame = frame.copy()
  
  def stream_thread_func(self):
      """Background thread to stream frames to client"""
      rate = rospy.Rate(10) 
      
      while not rospy.is_shutdown():
        if self.client_socket is None:
          rate.sleep()
          continue
            
        with self.lock:
          frame = self.latest_frame.copy() if self.latest_frame is not None else None
        
        if frame is None:
          rate.sleep()
          continue
            
        _, jpeg = cv2.imencode('.jpg', frame)
        data = jpeg.tobytes()
        size = len(data).to_bytes(4, byteorder='big')
        
        # Send frame
        try:
          self.client_socket.sendall(size + data)
        except Exception as e:
          rospy.logerr(f"Streaming error: {e}")
          self.client_socket = None
        
        rate.sleep()
  
  def start_streaming(self):
    """Start the streaming thread"""
    self.stream_thread = threading.Thread(target=self.stream_thread_func)
    self.stream_thread.daemon = True
    self.stream_thread.start()
    rospy.loginfo("Camera streaming started")
