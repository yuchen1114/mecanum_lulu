print("Script started")
import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

if not cap.isOpened():
    print("Camera failed to open!")
    exit(1)

ret, frame = cap.read()
print("ret:", ret, "| frame is None:", frame is None)
if ret and frame is not None:
    cv2.imwrite('snapshot_1280x720.jpg', frame)
    print("Saved snapshot_1280x720.jpg")
else:
    print("Failed to capture image!")

cap.release()

