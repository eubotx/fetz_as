import numpy as np
import cv2

class DiffDetector:
    def __init__(self):
        self.static_back = None
        self.img = None

    def track(self, image_gray, debug_image=None):
        # Our operations on the frame come here
        image_gray = cv2.cvtColor(debug_image, cv2.COLOR_BGR2GRAY)
        image_gray = cv2.GaussianBlur(image_gray, (11, 11), 25)
        if self.static_back is None:
            self.static_back = image_gray
            return

        # cv2.imshow('static_back', self.static_back)

        # Difference between static background
        # and current frame(which is GaussianBlur)
        diff_frame = cv2.absdiff(self.static_back, image_gray)
        # cv2.imshow('diff_frame', diff_frame)

        # If change in between static background and
        # current frame is greater than 30 it will show white color(255)
        thresh_frame = cv2.threshold(diff_frame, 30, 255, cv2.THRESH_BINARY)[1]
        # thresh_frame = cv2.dilate(thresh_frame, None, iterations=2)
        cv2.imshow('DiffDetector: thresh_frame', thresh_frame)

        # Finding contour of moving object
        cnts, _ = cv2.findContours(thresh_frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        print(f"Number of contours: {len(cnts)}")
        individual_contours = []
        for contour in cnts:
            if cv2.contourArea(contour) < 10 * 10:
                continue

            rect = np.array(cv2.boundingRect(contour))
            (x, y, w, h) = rect
            # print(f"Bounding box: x,y:{x}, {y}; w,h:{w}, {h}")
            # Draw bounding box rectangle in red (not rotated)
            if debug_image is not None:
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 0, 255), 1)

            # Draw min area rectangle in blue
            # rect = cv2.minAreaRect(contour)
            # cv2.drawContours(frame, [cv2.boxPoints(rect).astype(int)], -1, (255, 0, 0), 3)

            ###
            individual_contours.append(rect)
            i, j = (0, 1)
            while i < len(individual_contours):
                while j < len(individual_contours):
                    inter = self.intersect(individual_contours[i], individual_contours[j])
                    if inter is not None:
                        individual_contours[i] = inter
                        individual_contours.pop(j)
                        break
                    j += 1
                if j >= len(individual_contours):
                    i += 1
                    j = i + 1

        if debug_image is not None:
            # Draw individual contours in yellow
            for individual_contour in individual_contours:
                (x, y, w, h) = individual_contour
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), (0, 255, 255), 1)

            # # Draw all contours in green
            # cv2.drawContours(frame, cnts, -1, (0, 255, 0), 3)

        self.static_back = image_gray

        return {"boxDetections": individual_contours, "debug_image": debug_image}

    # define a function that checks if two rectangles defined by (x,y,w,h) intersect
    def intersect(self, r1, r2, expand=10):
        def expand_rect(r, e):
            return np.array([r[0] - e, r[1] - e, r[2] + e, r[3] + e])
        r1 = expand_rect(r1, expand)
        r2 = expand_rect(r2, expand)
        x1, y1, w1, h1 = r1
        x2, y2, w2, h2 = r2
        if x1 < x2 + w2 and x1 + w1 > x2 and y1 < y2 + h2 and y1 + h1 > y2:
            return np.array([min(x1,x2), min(y1,y2), max(-x1 + x2 + w2, x1 + w1 - x2), max(-y1 + y2 + h2, y1 + h1 - y2)])
        return None

    def area(self, r):
        return r[2] * r[3]