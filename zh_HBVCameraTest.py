import cv2

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("Unable to open camera!")
    exit()

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('output.avi', fourcc, 20.0, (320, 240))

while True:
    ret, frame = cap.read()

    if not ret:
        print("No Frames!")
        break

    out.write(frame)

    cv2.imshow('Live Stream', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
