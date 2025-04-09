import socket
import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("yolov8n.pt")  # You can use your fine-tuned model here

HOST = '0.0.0.0'
PORT = 9999

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("[INFO] Waiting for connection...")
conn, addr = server_socket.accept()
print(f"[INFO] Connection from {addr}")

def recvall(sock, size):
    data = b''
    while len(data) < size:
        packet = sock.recv(size - len(data))
        if not packet:
            return None
        data += packet
    return data

while True:
    raw_len = recvall(conn, 4)
    if not raw_len:
        break
    frame_len = int.from_bytes(raw_len, byteorder='big')
    frame_data = recvall(conn, frame_len)

    frame = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
    results = model(frame)[0]

    for box in results.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        label = model.names[cls_id]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'{label} {conf:.2f}', (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("YOLOv8 Inference", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

conn.close()
server_socket.close()
