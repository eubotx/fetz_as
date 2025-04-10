import json
import numpy as np
import spatialmath as sm
from src.detection.calibration.calibration import Calibration

class DoubleFisheye:
    """
    Class to handle the double fisheye camera model.
    Based on:
        The Double Sphere Camera Model
        Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers
        Technical University of Munich
        https://arxiv.org/pdf/1807.08957
    """

    def __init__(self, camera_matrix, dist_coeffs):
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        self.xi = dist_coeffs[0]
        self.alpha = dist_coeffs[1]

    def load(self, options):
        self.calib = Calibration(options['camera_calibration_path']).get()

    def huber_loss(self, x):
        """
        Huber loss function for robust optimization.
        :param x: Input value.
        :return: Huber loss.
        """
        delta = 1.0
        absX = np.abs(x)
        if absX <= delta:
            return 0.5 * x * x
        else:
            return delta * (absX - 0.5 * delta)

    def calibrate(self, image_points, object_points):
        assert(image_points.shape[0] == object_points.shape[0])
        num_detections = object_points.shape[0]
        num_points_per_detection = object_points.shape[1]

        # Flatten image_points
        image_points = image_points.reshape(-1,2).transpose()
        # Convert image points to SE3
        object_points = [[sm.SE3(object_corner) for object_corner in object_corners] for object_corners in object_points]

        intrinsics = np.zeros([6,1]) # [fx, fy, cx, cy, xi, alpha]
        initialSe3 = sm.SE3(np.ones(3))
        posesCamFromCalibTarget = [initialSe3 for _ in range(num_detections)]

        calibTargetInCam = [camFromCalibTarget * object_corner
                     for camFromCalibTarget, object_corners in zip(posesCamFromCalibTarget, object_points)
                     for object_corner in object_corners]
        assert(len(calibTargetInCam) == num_detections * num_points_per_detection)
        residual = self.project(calibTargetInCam) - image_points

        self.projectJacob(calibTargetInCam)
        self.posesJacob(object_points, posesCamFromCalibTarget)

    def posesJacob(self, pointsInCalibTarget, posesCamFromCalibTarget):
        calibTargetInCam = [camFromCalibTarget.jacobian()  * calib_corner
                     for camFromCalibTarget, calib_corners in zip(posesCamFromCalibTarget, pointsInCalibTarget)
                     for calib_corner in calib_corners]


    # img_points2, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, distortion_coeffs)
    # points is 3xN
    def project(self, points):
        if isinstance(points, np.ndarray):
            assert(points.shape[0] == 3)
        elif isinstance(points, list) and isinstance(points[0], sm.SE3):
            points = np.array([point.t for point in points]).transpose()
        N = points.shape[1]
        d1 = np.linalg.norm(points, axis=0)
        xiD1Z = self.xi * d1 + points[2,:]
        d2 = np.linalg.norm(np.vstack([points[0, :], points[1, :], xiD1Z]), axis=0)
        uv = np.zeros([2, N])
        denominator = self.alpha * d2 + (1-self.alpha) * xiD1Z
        uv[0, :] = self.fx * points[0, :] / denominator + self.cx
        uv[1, :] = self.fy * points[1, :] / denominator + self.cy
        return uv

    def projectJacob(self, pointsInCam):
        d1 = np.linalg.norm(pointsInCam, axis=0)
        xiD1Z = self.xi * d1 + pointsInCam[2, :]
        d2 = np.linalg.norm(np.vstack([pointsInCam[0, :], pointsInCam[1, :], xiD1Z]), axis=0)
        xiD1Z = self.xi * d1 + pointsInCam[2, :]
        denominator = self.alpha * d2 + (1-self.alpha) * xiD1Z
        denominator2 = denominator * denominator

        dd1Dx = pointsInCam[0, :] / d1
        dd1Dy = pointsInCam[1, :] / d1
        dd1Dz = pointsInCam[2, :] / d1
        dd2Dx = 1/d2 * (pointsInCam[0, :] + (self.xi * d1 + pointsInCam[2, :]) * self.xi * dd1Dx)
        dd2Dy = 1/d2 * (pointsInCam[1, :] + (self.xi * d1 + pointsInCam[2, :]) * self.xi * dd1Dy)
        dd2Dz = 1/d2 * ((self.xi * d1 + pointsInCam[2, :]) * (self.xi * dd1Dz + 1))

        dDenominatorDx = self.alpha * dd2Dx + (1-self.alpha) * self.xi * dd1Dx
        dDenominatorDy = self.alpha * dd2Dy + (1-self.alpha) * self.xi * dd1Dy
        dDenominatorDz = self.alpha * dd2Dz + (1-self.alpha) * self.xi * (dd1Dz + 1)
        dDenominatorDXi = (1-self.alpha) * d1

        dPiuDx = (self.fx * denominator - self.fx * pointsInCam[0, :] * dDenominatorDx) / denominator2
        dPiuDy = - self.fx * pointsInCam[0, :] * dDenominatorDy / denominator2
        dPiuDz = - self.fx * pointsInCam[0, :] * dDenominatorDz / denominator2

        dPivDx = - self.fy * pointsInCam[1, :] * dDenominatorDx / denominator2
        dPivDy = (self.fy * denominator - self.fy * pointsInCam[1, :] * dDenominatorDy) / denominator2
        dPivDz = - self.fy * pointsInCam[1, :] * dDenominatorDz / denominator2

        dPiDc = np.eye(2)
        dPiDf = np.diag([pointsInCam[0, :] / denominator, pointsInCam[1, :] / denominator])
        piNumerator = np.array([[self.fx * pointsInCam[0, :]], [self.fy * pointsInCam[1, :]]])
        dPiDxi = piNumerator * ((self.alpha-1) * d1 / denominator2)
        dPiDalpha = piNumerator * (self.xi * d1 - d2) / denominator2

        # intrinsics = [fx, fy, cx, cy, xi, alpha]
        dPiDintrinsics = np.array([[dPiDf, dPiDc, dPiDxi, dPiDalpha]])
        return dPiDintrinsics

    def unproject(self, uv):
        assert(uv.shape[0] == 2)

        mx = (uv[0, :] - self.cx)/self.fx
        my = (uv[1, :] - self.cy)/self.fy
        r2 = mx * mx + my * my
        mz = (1 - self.alpha * r2) / (1 - self.alpha + self.alpha * np.sqrt(1 - (2*self.alpha - 1)*r2))

        mz2 = mz * mz
        scalar = (mz * self.xi + np.sqrt(mz2 + (1-self.xi*self.xi)*r2))/(mz2 + r2)
        points = np.vstack([mx, my, mz])*scalar + np.matrix([0, 0, self.xi]).transpose()
        return points


    def undistort(self, image):
        """
        Undistort an image using the camera matrix and distortion coefficients.

        :param image: Input image.
        :return: Undistorted image.
        """
        # Implement undistortion logic here
        pass

df = DoubleFisheye(np.array([[1,0,0],[0,1,0],[0,0,1]]), np.array([0,0]))

# df.project(np.random.rand(3,100))
# df.unproject(np.random.rand(2,100))

with open('src/detection/data/USBGS720P02-L170_calibration_points.json', 'r') as f:
    json_data = json.load(f)
    calib_data = {key: np.array(value) for key, value in json_data.items()}

df.calibrate(calib_data['img_points'], calib_data['obj_points'])