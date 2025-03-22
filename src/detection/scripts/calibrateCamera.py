import cv2
import numpy as np
import pyapriltags as apriltag

# Define the real-world size of the AprilTag (in meters or any unit)
APRILTAG_SIZE = 0.150

# Define object points for a single AprilTag
object_points = np.array([
    [-APRILTAG_SIZE / 2, -APRILTAG_SIZE / 2, 0],
    [APRILTAG_SIZE / 2, -APRILTAG_SIZE / 2, 0],
    [APRILTAG_SIZE / 2, APRILTAG_SIZE / 2, 0],
    [-APRILTAG_SIZE / 2, APRILTAG_SIZE / 2, 0]
], dtype=np.float32)

# Storage for detected corners and corresponding object points
obj_points = []  # Real-world 3D coordinates
img_points = []  # Image 2D coordinates

# Initialize the AprilTag detector
detector = apriltag.Detector(families="tagStandard41h12")

# Capture images (or load from a directory)
cap = cv2.VideoCapture(2)

print("Capturing calibration images. Press 'c' to capture, 'q' to finish.")

image_count = 0
resolution = None
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not capture frame.")
        break

    if resolution is None:
        resolution = frame.shape
        camera_frame_detections = np.zeros((int(resolution[0]), int(resolution[1]), 3), dtype=np.uint8)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    for detection in detections:
        # Draw detected tag corners
        for i in range(4):
            pt1 = tuple(detection.corners[i].astype(int))
            pt2 = tuple(detection.corners[(i+1) % 4].astype(int))
            cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

        center = tuple(detection.center.astype(int))
        cv2.circle(frame, center, 5, (0, 0, 255), -1)
        cv2.putText(frame, f"ID: {detection.tag_id}", (center[0] - 10, center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    cv2.imshow("AprilTag Detection", frame)

    key = cv2.waitKey(10) & 0xFF
    if key == ord('c'):  # Capture frame
        if not len(detections):
            print("Error: No AprilTag detected.")
            continue
        for detection in detections:
            # Store detected image points
            img_points.append(detection.corners.astype(np.float32))
            obj_points.append(object_points)
            for corner in detection.corners:
                cv2.circle(camera_frame_detections, corner.astype(int), 5, (0, 255, 0), -1)
        image_count += 1
        print(f"Captured image {image_count}")
        cv2.imshow("Camera Detections", camera_frame_detections)
    elif key == ord('q'):  # Quit capturing
        break

cap.release()
cv2.destroyAllWindows()

# Ensure we have enough points
print(f"Captured {image_count} images.")
print(f"img_points: {img_points}")
print(f"obj_points: {obj_points}")

if len(obj_points) < 10:
    print("Error: Not enough images captured for calibration.")
    exit()

# Perform camera calibration
ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)

# Display calibration results
print("\nCamera Calibration Results:")
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", distortion_coeffs)

# Calculate reprojection error
mean_error = 0
camera_frame_reprojection_error = np.zeros((resolution[0], resolution[1], 3), dtype=np.uint8)
for i in range(len(obj_points)):
    img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs)
    error = cv2.norm(img_points[i], img_points2.squeeze(), cv2.NORM_L2) / len(img_points2)
    # function to map 0 to green and 1 to red
    def color_map(x):
        return (0, 255 * (1-x), 255 * x)
    for corner in img_points2:
        cv2.circle(camera_frame_reprojection_error, corner.squeeze().astype(int), 1, color_map(error), -1)
    mean_error += error
print(f"Mean Error: {mean_error / len(obj_points)}")
cv2.imshow("Camera Reprojection Error", camera_frame_reprojection_error)
cv2.waitKey()

# Save calibration results
np.savez("camera_calibration_apriltag.npz",
         resolution=resolution,
         camera_matrix=camera_matrix,
         distortion_coeffs=distortion_coeffs,
         mean_error=mean_error)

print("Calibration data saved as 'camera_calibration_apriltag.npz'.")
cv2.destroyAllWindows()
