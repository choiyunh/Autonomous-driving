import cv2

img_color = cv2.imread('./img/b.png')
img_color = img_color[400:480, 120:520]  # y, x
img_gray = cv2.cvtColor(img_color, cv2.COLOR_BGR2GRAY)
ret, img_binary = cv2.threshold(img_gray, 127, 255, 0)
contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

for cnt in contours[:]:
    if 7500 < cv2.contourArea(cnt) < 7700:
        print(cnt)
        cv2.drawContours(img_color, [cnt], 0, (255, 0, 0), 3)  # blue
        print(cv2.contourArea(cnt))
print(img_gray[40,200])

cv2.imshow("result", img_color)

cv2.waitKey(0)
