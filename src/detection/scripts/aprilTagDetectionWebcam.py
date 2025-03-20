import cv2
import detector
import frame_source

if __name__ == "__main__":
    aprilTagDetector = detector.AprilTagDetector('../data/tiago_laptop_webcam_calibration.npz')
    webCamSource = frame_source.WebCamSource()

    while True:
        frame = webCamSource.get_frame()
        if frame is None:
            break

        # Convert frame to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect AprilTags
        detector_output = aprilTagDetector.detect(gray, frame)
        aprilTagDetections = detector_output["aprilTags"]
        print(f"Detected AprilTags: {len(aprilTagDetections)}.")

        # Show the frame
        cv2.imshow("AprilTagDetector Debug Output", detector_output["debug_image"])

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
