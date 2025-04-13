import socket
import cv2
import numpy as np

ROBOT_IP = '192.168.0.0' # change based on robot ip
PORT = 9999

def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Connecting to robot at {ROBOT_IP}:{PORT}...")
    client_socket.connect((ROBOT_IP, PORT))
    print("Connected!")
    
    def recvall(sock, size):
        data = b''
        while len(data) < size:
            packet = sock.recv(size - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    try:
        while True:
            raw_len = recvall(client_socket, 4)
            if not raw_len:
                break
            frame_len = int.from_bytes(raw_len, byteorder='big')
            frame_data = recvall(client_socket, frame_len)
            frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            cv2.imshow("Robot Camera Feed", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
