import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np

filepath = "/Users/ttorres/Documents/robofetz/test_data/live-2024-05m26s.png"

img = cv.imread(filepath)
scale = 0.5
height = int(img.shape[0] * scale)
width = int(img.shape[1] * scale)
img = cv.resize(img, (width, height))

cv.imshow("Original image", img)

copy_img = np.copy(img)
copy_img = cv.cvtColor(copy_img, cv.COLOR_BGR2RGB)
gray = cv.cvtColor(copy_img, cv.COLOR_RGB2GRAY)

# gray_blur = cv.GaussianBlur(gray,(11,11),0)
# gray_blur = cv.GaussianBlur(gray,(3,3),0)
gray_blur = gray.copy()
cv.imshow('Gaussian Blur', gray_blur)

# kernel = np.ones((5,5), np.uint8)
# kernel = np.ones((1,1), np.uint8)
# img_erosion = cv.erode(gray_blur, kernel, iterations=1)
img_erosion = gray_blur.copy()
cv.imshow('After erode', img_erosion)


lower = 100
upper = 500
adges = cv.Canny(img_erosion, lower,upper)
cv.imshow('Canny edge', adges)

kernel2 = np.ones((5,5), np.uint8)
img_dilation = cv.dilate(adges, kernel2, iterations=1)
cv.imshow('Dilated image', img_dilation)

cv.waitKey()
