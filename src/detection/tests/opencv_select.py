import cv2
import numpy as np

def do_nothing():
    pass

# create slider here
cv2.namedWindow("Slider")
cv2.resizeWindow("Slider", 640, 480)
cv2.createTrackbar("Hue Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Hue Max", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Saturation Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Saturation Max", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Value Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Value Max", "Slider", 0, 255, do_nothing)

filepath = "/Users/ttorres/Documents/robofetz/test_data/live-2024-05m26s.png"

img = cv2.imread(filepath)
img = cv2.resize(img, (640, 480))


while True:
    hue_min = cv2.getTrackbarPos("Hue Min", "Slider")
    hue_max = cv2.getTrackbarPos("Hue Max", "Slider")
    sat_min = cv2.getTrackbarPos("Saturation Min", "Slider")
    sat_max = cv2.getTrackbarPos("Saturation Max", "Slider")
    val_min = cv2.getTrackbarPos("Value Min", "Slider")
    val_max = cv2.getTrackbarPos("Value Max", "Slider")
    print(hue_min, hue_max, sat_min, sat_max, val_min, val_max)

    # set bounds
    lower_bound = np.array([hue_min, sat_min, val_min])
    upper_bound = np.array([hue_max, sat_max, val_max])

    # convert to HSV image
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # create mask
    mask = cv2.inRange(hsv_img, lower_bound, upper_bound)
    resulting_img = cv2.bitwise_and(img, img, mask=mask)

    stacked_imgs = np.hstack([img, resulting_img])

#     create a stacked image of the original and the HSV one.
    cv2.imshow("Image", stacked_imgs)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cv2.destroyAllWindows()

