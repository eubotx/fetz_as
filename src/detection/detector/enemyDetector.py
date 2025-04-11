
import numpy as np
import cv2

import detector

arrow_forward_x = 0.2 * np.array([[0, 0, 0],
                [1, 0, 0],
                [0.75, 0.25, 0],
                [1, 0, 0],
                [0.75, -0.25, 0],
                [1, 0, 0]])

def draw_outline(outline, image, text=None, color=(0, 255, 0)):
    for i in range(len(outline)):
        pt1 = tuple(outline[i].astype(int))
        pt2 = tuple(outline[(i + 1) % len(outline)].astype(int))
        cv2.line(image, pt1, pt2, color, 2)
        cv2.putText(image, f"{i}", pt1,
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    if text:
        cv2.putText(image, text, outline[0].astype(int),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

class EnemyDetector():
    def __init__(self, options: dict):
        self.diff_detector = detector.DiffDetector()
        self.detection_last = None

    def detect(self, image_gray, robot_detection, debug_image=None):
        bounding_rects = self.diff_detector.track(image_gray.copy(),
                                                        robot_detection['robotOutlineInCamera2D'] if robot_detection is not None else None,
                                                        debug_image)
        def get_contour_largest_area(contours):
            if len(contours) == 0:
                return None
            largest_area = 0.0
            largest_ind = 0
            print(f'Contours: {contours}')
            for i in range(len(contours)):
                print(f'Contours[i]: {contours[i]}')
                (x, y, w, h) = contours[i]
                area = w * h
                if area > largest_area:
                    largest_ind = i
                    largest_area = area
            return np.array(contours[largest_ind])

        enemy_contour = get_contour_largest_area(bounding_rects)
        if enemy_contour is None:
            if debug_image is not None:
                if self.detection_last is not None:
                    cv2.circle(debug_image, self.detection_last['enemyInCamera2D'].astype(int), 5, (0, 0, 255), 2)
                cv2.imshow("Enemy pos", debug_image)
            return self.detection_last

        detection = {}
        detection['enemyBBoxInCamera2D'] = enemy_contour
        print(f'enemyContour {enemy_contour}')
        (x, y, w, h) = enemy_contour
        detection['enemyInCamera2D'] = np.array([x + w / 2, y + h / 2])

        if debug_image is not None:
            print(f'enemyInCamera2D: {detection["enemyInCamera2D"].astype(int)}')
            cv2.circle(debug_image, detection['enemyInCamera2D'].astype(int), 5, (0, 255, 0), 2)
            cv2.imshow("Enemy pos", debug_image)

        self.detection_last = detection
        return detection
