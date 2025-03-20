#!/usr/bin/env python3

import cv2

import detector
import frame_source

# use system plotting
# plt.switch_backend('macosx')

if __name__ == '__main__':
    # frame_source = frame_source.VideoFileSource()
    frame_source = frame_source.WebCamSource()

    diff_detector = detector.DiffDetector()
    april_tag_detector = detector.AprilTagDetector()
    while True:
        frame = frame_source.get_frame()
        if frame is None:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        april_tag_output = april_tag_detector.detect(gray, frame.copy())
        april_tag_detections = april_tag_output["aprilTags"]
        print(f"Detected AprilTags: {len(april_tag_detections)}.")
        if isinstance(april_tag_output, dict) and april_tag_output["debug_image"] is not None:
            cv2.imshow("AprilTagDetector Debug Image", april_tag_output["debug_image"])

        # Detect differences in the image
        diff_detector_output = diff_detector.track(gray, frame.copy())
        if isinstance(diff_detector_output, dict) and diff_detector_output['debug_image'] is not None:
            cv2.imshow("DiffDetector Debug Image", diff_detector_output['debug_image'])

        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cv2.destroyAllWindows()

