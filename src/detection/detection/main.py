#!/usr/bin/env python3

import cv2
import numpy as np

import detector
import frame_source

from math_funcs import Pose

# use system plotting
# plt.switch_backend('macosx')

# Define the real-world size of the AprilTag (in meters or any unit)
APRILTAG_SIZE = 0.150
DETECTION_SCALE_RATIO = 5 / 9  # Detection is only on the inside area while measurement was taken on outside of tag
DETECTION_SCALE_RATIO = 1

# Define object points for a single AprilTag
april_tag_points = np.array([
    [0, APRILTAG_SIZE, 0],
    [APRILTAG_SIZE, APRILTAG_SIZE, 0],
    [APRILTAG_SIZE, 0, 0],
    [0, 0, 0]
], dtype=np.float32)
april_tag_points = DETECTION_SCALE_RATIO * (april_tag_points - np.array([APRILTAG_SIZE/2, APRILTAG_SIZE/2, 0]))

if __name__ == '__main__':
    options = {
        # Calibration
        'camera_calibration_path': 'src/detection/data/USBGS720P02-L170_calibration.json',
        # ROS
        'publish_ros': False,
        # Video
        # 'frame_path': '/home/tiago/Documents/CamTestFootage/vids_position_top_down/output_bot_april_corners.mp4',
        'frame_path': 'src/detection/recordings/output_april_corner_movement.mp4',
        # Webcam
        'webcam_id': 2,
        'webcam_save_stream_path': 'src/detection/recordings/testSeqxxx.mp4',
        # Tags
        'arena_tag': {'id': 2, 'family': 'tagStandard41h12'},
        'robot_tag': {'id': 12, 'family': 'tagStandard41h12'},
    }

    # frame_source = frame_source.GenericSource(frame_source.SourceType.Video, options=options)
    frame_source = frame_source.GenericSource(frame_source.SourceType.Webcam, options=options)


    ros_publisher = None

    diff_detector = detector.DiffDetector()
    april_tag_detector = detector.AprilTagDetector(frame_source.get_calibration())
    arena_detector = None
    cameraFromWorld = Pose()
    while True:
        print('Getting frame...')
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

        if arena_detector is None and 'arena_tag' in options.keys():
            # Initialize the arena detector and estimate arena pose
            arena_detector = detector.ArenaDetector(frame)
            arena_tag = options['arena_tag']
            cameraFromWorld = None
            for detection in april_tag_detections:
                if detection.tag_family.decode() == arena_tag['family'] and detection.tag_id == arena_tag['id']:
                    cameraFromWorld = Pose(detection.pose_R, detection.pose_t)
                    print(f"Detected arena tag: {arena_tag['family']}-{arena_tag['id']}. Defined cameraFromWorld:\n{cameraFromWorld}")
            if not cameraFromWorld:
                print(f"Could not find arena tag: {arena_tag['family']}-{arena_tag['id']}. Retrying...")
                arena_detector = None
            continue

        print(f'Detecting robot...')

        # Detect Robot
        robot_tag = options['robot_tag']
        robot_shape = np.array([[0,0,0], [1,0,0], [0.75, 0.5, 0], [0.25, 0.5, 0], [0, 0, 0]])
        for detection in april_tag_detections:
            if detection.tag_family.decode() == robot_tag['family'] and detection.tag_id == robot_tag['id']:
                robot_img = frame.copy()
                if detection.pose_err > 1e-3:
                    print(f"Detection error larger than: {detection.pose_err:.3f} m")

                calibration = frame_source.get_calibration()
                projectedPoint = []
                for i in range(len(april_tag_points)):
                    # detection is cameraFromTag
                    aprilInCamera = detection.pose_R @ april_tag_points[i].reshape(-1,1) + detection.pose_t
                    camFromWorld_R = np.eye(3)
                    camFromWorld_t = np.zeros(3)
                    projectedPoint.append(calibration.project(aprilInCamera))
                    print(f'Projected point in: {projectedPoint}, detection: {detection.corners[i]}')

                for i in range(4):
                    # Tag
                    pt1 = tuple(detection.corners[i].astype(int))
                    pt2 = tuple(detection.corners[(i + 1) % 4].astype(int))
                    cv2.line(robot_img, pt1, pt2, (0, 255, 0), 2)
                    cv2.putText(robot_img, f"{i}", pt1,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    # Reprojections
                    pt1 = tuple(projectedPoint[i].squeeze().astype(int))
                    pt2 = tuple(projectedPoint[(i + 1) % 4].squeeze().astype(int))
                    cv2.line(robot_img, pt1, pt2, (255, 0, 0), 2)
                    cv2.putText(robot_img, f"{i}", pt1,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                # Draw the center of the tag
                center = tuple(detection.center.astype(int))
                cv2.circle(robot_img, center, 5, (0, 0, 255), -1)

                # Display tag ID
                cv2.putText(robot_img, f"ID: {detection.tag_id}", (center[0] - 10, center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                cv2.imshow("RobotDetector Debug Image", robot_img)

        # Detect differences in the image
        diff_detector_output = diff_detector.track(gray, frame.copy())
        if isinstance(diff_detector_output, dict) and diff_detector_output['debug_image'] is not None:
            cv2.imshow("DiffDetector Debug Image", diff_detector_output['debug_image'])

        if options['publish_ros']:
            if ros_publisher is None:
                import publisher
                ros_publisher = publisher.RosPublisher()
                ros_publisher.publish({'string': 'this is sent on startup', 'test': 0, 'aaa': False})
            else:
                ros_publisher.publish({'string': 'this is sent after', 'test': 0, 'aaa': False})

        if cv2.waitKey(1) == ord('q'):
            break
    # When everything done, release the capture
    cv2.destroyAllWindows()

