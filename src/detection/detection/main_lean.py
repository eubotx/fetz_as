import cv2
import numpy as np
import json
import frame_source
import detector
import math

# Load transformation data from the saved JSON file
def load_transformation_data():
    with open("src/detection/data/transformation_data.json", "r") as f:
        data = json.load(f)
    src_points = np.float32(data["src_points"])
    matrix = np.float32(data["matrix"])
    return src_points, matrix

# Global variables for transformation
src_points, matrix = load_transformation_data()
arena_width = 450  # cm (width of the arena)
arena_height = 450  # cm (height of the arena)

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

frame_source = frame_source.GenericSource(frame_source.SourceType.Webcam, options=options)
diff_detector = detector.DiffDetector()
april_tag_detector = detector.AprilTagDetector(frame_source.get_calibration())


#Function to draw the forward direction axis (X-axis)
def draw_forward_direction_axis(frame, center, yaw, length=50):
    # Calculate the forward direction using yaw (angle) in radians
    forward_direction = (math.cos(yaw), math.sin(yaw))

    # Scale the forward direction vector
    forward_end = (int(center[0] + forward_direction[0] * length),
                   int(center[1] - forward_direction[1] * length))  # Invert y-axis for screen coordinates

    # Draw the forward direction (X-axis) on the frame
    cv2.arrowedLine(frame, center, forward_end, (0, 255, 0), 5)  # Green forward direction (X-axis)

# Main loop for live video feed
while True:
    # Capture frame from the webcam
    #print('Getting frame...')
    frame = frame_source.get_frame()
    if frame is None:
        break
    
    transformed_frame = cv2.warpPerspective(frame, matrix, (arena_width, arena_height))
    gray = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2GRAY)
    

    # Display the transformed image
    cv2.imshow("Transformed Arena", transformed_frame)

    # Show the original frame
    cv2.imshow("Original Arena", frame)

    # Detect AprilTags
    april_tag_output = april_tag_detector.detect(gray, transformed_frame.copy())
    april_tag_detections = april_tag_output["aprilTags"]
    #print(f"Detected AprilTags: {len(april_tag_detections)}.")
    if isinstance(april_tag_output, dict) and april_tag_output["debug_image"] is not None:
        cv2.imshow("AprilTagDetector Debug Image", april_tag_output["debug_image"])

    # Detect Robot
    robot_tag = options['robot_tag']
    robot_shape = np.array([[0,0,0], [1,0,0], [0.75, 0.5, 0], [0.25, 0.5, 0], [0, 0, 0]])
    for detection in april_tag_detections:
        if detection.tag_family.decode() == robot_tag['family'] and detection.tag_id == robot_tag['id']:
            
            robot_img = transformed_frame.copy()
            # Draw the center of the tag
            center = tuple(detection.center.astype(int))
            cv2.circle(robot_img, center, 5, (0, 0, 255), -1)
            

            R = detection.pose_R

            yaw =  math.atan2(R[1, 0], R[0, 0])
            offset_yaw = yaw - math.pi/2
            print(f"Robot Yaw: {offset_yaw} radians")

            # Optionally: Display the angle of the forward direction relative to the camera frame
            forward_angle = math.atan2(R[1, 0], R[0, 0])
            cv2.putText(robot_img, f"Forward Angle: {offset_yaw:.2f}°", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            # Draw the forward direction (X-axis) on the robot image
            draw_forward_direction_axis(robot_img, center, -offset_yaw)


            cv2.imshow("RobotDetector Debug Image", robot_img)



    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close all OpenCV windows
cv2.destroyAllWindows()
