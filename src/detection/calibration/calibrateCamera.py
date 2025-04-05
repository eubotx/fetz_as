import cv2
import json
import numpy as np
import pyapriltags as apriltag

# Define the real-world size of the AprilTag (in meters or any unit)
APRILTAG_SIZE = 0.150
DETECTION_SCALE_RATIO = 5 / 9  # Detection is only on the inside area while measurement was taken on outside of tag

# Define object points for a single AprilTag
april_tag_points = np.array([
    [0, 0, 0],
    [0, APRILTAG_SIZE, 0],
    [APRILTAG_SIZE, APRILTAG_SIZE, 0],
    [APRILTAG_SIZE, 0, 0]
], dtype=np.float32)
april_tag_points *= DETECTION_SCALE_RATIO

def detect_points():
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
            resolution = frame.shape[0:2]
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
                obj_points.append(april_tag_points)
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

    # Save points as backup
    np.savez("calibration_points.npz",
             image_count=image_count,
             resolution=resolution,
             img_points=img_points,
             obj_points=obj_points,
             april_tag_points=april_tag_points)

    return resolution, img_points, obj_points

def compute_calibration(resolution, img_points, obj_points):
    useFisheyeModel = False
    if not useFisheyeModel:
        ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objectPoints=obj_points,
            imagePoints=img_points,
            imageSize=resolution,
            cameraMatrix=None,
            distCoeffs=None,
            rvecs=None,
            tvecs=None,
            flags=cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL + cv2.CALIB_TILTED_MODEL,
            # flags=cv2.CALIB_FIX_PRINCIPAL_POINT + cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_THIN_PRISM_MODEL +cv2.CALIB_TILTED_MODEL,
            # criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
    else:
        camera_matrix = np.zeros((3, 3))
        distortion_coeffs = np.zeros((14, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float32) for i in range(len(obj_points))]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float32) for i in range(len(obj_points))]
        ret, _, _, _, _ = cv2.fisheye.calibrate(
            objectPoints=obj_points.reshape((-1, 1, 3)).astype(np.float32),
            imagePoints=img_points.reshape((-1, 1, 2)).astype(np.float32),
            image_size=resolution,
            K=camera_matrix,
            D=distortion_coeffs,
            rvecs=rvecs,
            tvecs=tvecs,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))

    # Display calibration results
    print("\nCamera Calibration Results:")
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", distortion_coeffs)

    # Calculate reprojection error
    mean_error = 0
    max_error = 0
    camera_frame_reprojection_error = np.zeros((resolution[0], resolution[1], 3), dtype=np.uint8)
    for i in range(len(obj_points)):
        if not useFisheyeModel:
            img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs)
        else:
            img_points2, _ = cv2.fisheye.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs)
        error = cv2.norm(img_points[i], img_points2.squeeze(), cv2.NORM_L2) / len(img_points2)
        for corner in img_points2:
            cv2.circle(camera_frame_reprojection_error, corner.squeeze().astype(int), 1, (0, 255 * (1-error), 255 * error), -1)
            cv2.putText(camera_frame_reprojection_error, f"{error:.2f}", corner.squeeze().astype(int),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 255 * (1-error), 255 * error), 1)
        mean_error += error
        max_error = max(max_error, error)
    mean_error /= len(obj_points)
    print(f"Mean Error: {mean_error}, Max Error: {max_error}")
    cv2.imshow("Camera Reprojection Error", camera_frame_reprojection_error)
    cv2.waitKey()

    # Save calibration results
    calibration_result = {
        "useFisheyeModel": useFisheyeModel,
        "resolution": resolution.squeeze().tolist(),
        "camera_matrix": camera_matrix.squeeze().tolist(),
        "distortion_coeffs": distortion_coeffs.squeeze().tolist(),
        "mean_error": mean_error,
        "max_error": max_error,
    }
    json_object = json.dumps(calibration_result, indent=4)
    with open("camera_calibration.json", "w") as outfile:
        outfile.write(json_object)
        print("Calibration data saved as 'camera_calibration.json'.")
    cv2.destroyAllWindows()

resolution = None
img_points = None
obj_points = None

LIVE_CALIBRATION = False
if LIVE_CALIBRATION:
    print(f'Running live calibration!')
    resolution, img_points, obj_points = detect_points()
else:
    print(f'Running calibration from matches in file!')
    detections_filepath = './../data/USBGS720P02-L170_calibration_points' + '.npz'
    with np.load(detections_filepath) as data:
        for k, v in data.items():
            print(f"{k}: {v}")
        img_points = data['img_points'][::5]
        obj_points = data['obj_points'][::5]
        if 'resolution' not in data.keys():     # Assume a default resolution in case one was not saved. THis is just a hack.
            resolution = np.array([480, 640])

    for i in range(len(obj_points)):
        obj_points[i] = april_tag_points
    print(obj_points)

    max_corner = np.zeros(2)
    for i, detection in enumerate(img_points):
        for j, corner in enumerate(detection):
            # img_points[i][j] = corner[::-1]
            max_corner = np.max([img_points[i][j], max_corner], axis=0)
    print(f"max_corner: {max_corner}")

compute_calibration(resolution, img_points, obj_points)

