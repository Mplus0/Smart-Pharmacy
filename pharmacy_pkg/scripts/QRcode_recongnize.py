import cv2
from pyzbar.pyzbar import decode

img = cv2.imread("NULL.jpg")
cap = cv2.VideoCapture("http://192.168.8.177:8080/stream?topic=/camera/rgb/image_raw")

data = ['link']

while True:
    success, img = cap.read()
    QR_code = decode(img)
    
    for QR in QR_code:
        QR_data = QR.data.decode("utf-8")
        print(QR_data)
    #     if (QR_data != data[-1]):
    #         data.append(QR_data)
    #         print(data)
    #     point = QR.rect
    #     cv2.rectangle(img, (point[0], point[1]), (point[0] + point[2], point[1] + point[3]), (200, 0, 200), 3)
    #     cv2.putText(img, QR_data, (point[0] - 50, point[1] - 5), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 255), 1)

    cv2.imshow("output", img)
    if cv2.waitKey(1) & 0xFF == 27:
         break