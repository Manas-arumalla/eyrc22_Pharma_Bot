import cv2

cap = cv2.VideoCapture('http://192.168.1.195:4747/video')

while True:
    ret, frame = cap.read()

    cv2.imshow("frame", frame)
    cv2.waitKey(1)