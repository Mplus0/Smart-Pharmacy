import cv2
import pytesseract


# 调用tesseract
# pytesseract.pytesseract.tesseract_cmd = r"D:\Talos\ROS\tesseract\tesseract.exe"
cap = cv2.VideoCapture(0)

data = ['化','验','区','血','常','规','体','液','免','疫','激','素','检','测','窗','口','空','闲','忙','碌','中']
result = []
output = '未检测'

while True:
    # 读取图片文件
    success, img = cap.read()
    # 转成灰度图片
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 二值化
    ret, img = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)

    cv2.rectangle(img, (100, 200), (750, 395), (0, 0, 255), 2)
    img_roi = img[200:395, 100:750]

    text = pytesseract.image_to_string(img_roi, lang="chi_sim_baidu")

    text = list(text)
    # print(text)
    # print(result)
    text_length = len(text)
    for i in range(text_length):
        if text[i] in data:
            if text[i] not in result:
                result.append(text[i])

    # 整理字符
    if "化" in result:
        if "空" in result:
            output = "化验区空闲中"
        if "忙" in result:
            output = "化验区忙碌中"
        print(output)
        result = []
    if "血" in result:
        if "空" in result:
            output = "血常规窗口空闲中"
        if "忙" in result:
            output = "血常规窗口忙碌中"
        print(output)
        result = []
    if "体" in result:
        if "空" in result:
            output = "体液窗口空闲中"
        if "忙" in result:
            output = "体液窗口忙碌中"
        print(output)
        result = []
    if "免" in result:
        if "空" in result:
            output = "免疫检测窗口空闲中"
        if "忙" in result:
            output = "免疫检测窗口忙碌中"
        print(output)
        result = []
    if "激" in result:
        if "空" in result:
            output = "激素检验窗口空闲中"
        if "忙" in result:
            output = "激素检验窗口忙碌中"
        print(output)
        result = []

    cv2.imshow("output", img)
    if cv2.waitKey(1) & 0xFF == 27:
        break




