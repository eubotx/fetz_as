#!/usr/bin/env python3

import timeit
import cv2
import numpy as np

import detector
import frame_source

from src.detection.math import Pose

# use system plotting
# plt.switch_backend('macosx')


class DetectionNode():
    def __init__(self):
        self.options = {
            # Calibration
            'camera_calibration_path': '/home/tiago/repos/fetz_as/src/detection/data/USBGS720P02-L170_calibration.json',
            # ROS
            'publish_ros': False,
            # Video
            # 'frame_path': '/home/tiago/Documents/CamTestFootage/vids_position_top_down/output_bot_april_corners.mp4',
            'frame_path': '/home/tiago/Documents/CamTestFootage/vids_position_top_down/output_april_corner_movement.mp4',
            # 'frame_path': '/home/tiago/Documents/CamTestFootage/vids_position_top_down/output_bot_april_corners.mp4',
            # 'frame_path': '/home/tiago/repos/fetz_as/src/detection/data/fetz2025-1.jpg',
            # 'frame_path': '/home/tiago/repos/fetz_as/src/detection/data/fetz2025-2.jpg',
            # 'frame_path': '/home/tiago/repos/fetz_as/src/detection/data/fetz2025-3.jpg',
            # 'frame_path': '/home/tiago/repos/fetz_as/src/detection/data/fetz2025-4.jpg',
            # 'frame_path': '/home/tiago/repos/fetz_as/src/detection/data/fetz2025-under-1.jpg',
        # Webcam
            'webcam_id': 0,
            'webcam_save_stream_path': '/home/tiago/repos/fetz_as/src/detection/data/testSeqxxx.mp4',
            # Tags
            'arena_tag': {'id': 2, 'family': 'tagStandard41h12', 'size': 0.15},
            'robot_tags': { 'sizes' : 0.125, 'family': 'tagStandard41h12',
                'top_id': 12,
                'bottom_id': 31},
            'robot_tags': {'sizes': 0.15, 'family': 'tagStandard41h12',
                           'top_id': 1,
                           'bottom_id': 20000}
        }

        self.frame_source = frame_source.GenericSource(frame_source.SourceType.Video, options=self.options)
        # self.frame_source = frame_source.GenericSource(frame_source.SourceType.Webcam, options=self.options)
        # self.frame_source = frame_source.GenericSource(frame_source.SourceType.Image, options=self.options)

        self.ros_publisher = None

        self.robot_detector = detector.RobotDetector(self.options, self.frame_source.get_calibration())
        self.enemy_detector = detector.EnemyDetector(self.options)
        self.april_tag_detector = detector.AprilTagDetector(self.frame_source.get_calibration())
        self.arena_detector = None
        self.cameraFromWorld = Pose()

    def process_frame(self):
        print('Getting frame...')
        frame = self.frame_source.get_frame()
        if frame is None:
            return
        # frame[300:480, 0:200] = 0
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect robot
        robot_detection = self.robot_detector.detect(gray, Pose(), frame.copy())
        enemy_detection = self.enemy_detector.detect(gray, robot_detection, frame.copy())


        # Detect AprilTags
        april_tag_output = self.april_tag_detector.detect(gray, 0.15)
        april_tag_detections = april_tag_output["aprilTags"]

        # if self.arena_detector is None and 'arena_tag' in self.options.keys():
        #     # Initialize the arena detector and estimate arena pose
        #     self.arena_detector = detector.ArenaDetector(frame)
        #     arena_tag = self.options['arena_tag']
        #     self.cameraFromWorld = None
        #     for detection in april_tag_detections:
        #         if detection.tag_family.decode() == arena_tag['family'] and detection.tag_id == arena_tag['id']:
        #             self.cameraFromWorld = Pose(detection.pose_R, detection.pose_t)
        #             print(f"Detected arena tag: {arena_tag['family']}-{arena_tag['id']}. Defined cameraFromWorld:\n{self.cameraFromWorld}")
        #     if not self.cameraFromWorld:
        #         print(f"Could not find arena tag: {arena_tag['family']}-{arena_tag['id']}. Retrying...")
        #         self.arena_detector = None
        #     return

        if self.options['publish_ros']:
            if self.ros_publisher is None:
                import publisher
                ros_publisher = publisher.RosPublisher()
                ros_publisher.publish({'string': 'this is sent on startup', 'test': 0, 'aaa': False})
            else:
                self.ros_publisher.publish({'string': 'this is sent after', 'test': 0, 'aaa': False})

        # if cv2.waitKey(1) == ord('q'):
        #     return

    def __del__(self):
        # When everything done, release the capture
        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = DetectionNode()

    while True:
        start_time = timeit.default_timer()
        node.process_frame()
        process_time_ms = 1e3*(timeit.default_timer() - start_time)
        print(f"Time to process frame: {process_time_ms:.3f} ms")
        if process_time_ms < 33:
            cv2.waitKey(33 - int(process_time_ms))
