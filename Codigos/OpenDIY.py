import cv2
img = cv2.imread("bus.jpg")
cv2.imshow("Output",img)
print(img[500,500,:])
cv2.waitKey(0)
