import cv2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

detector = cv2.QRCodeDetector()

while True:

    ret, frame = cap.read()
    if not ret:
        break
    
    data, bbox, _ = detector.detectAndDecode(frame)
    
    if bbox is not None:
        for i in range(len(bbox)):
            points = bbox[i].reshape(-1, 2).astype('int')
            for j in range(len(points)):
                pt1 = tuple(points[j])
                pt2 = tuple(points[(j + 1) % len(points)])
                cv2.line(frame, pt1, pt2, color=(0, 255, 0), thickness=2)
        
        if data:
            cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('QR Code Scanner', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
