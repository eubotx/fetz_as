#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;  // Define the json type

int main() {
    // Load objpoints and imgpoints from JSON file
    std::vector<std::vector<cv::Point3f>> objpoints;
    std::vector<std::vector<cv::Point2f>> imgpoints;

    json root;
    std::string calibDataPath = "src/detection/data/USBGS720P02-L170_calibration_points.json.json";
    {
        std::ifstream file(calibDataPath);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open: " << calibDataPath << std::endl;
            return -1;
        }

        // Parse the JSON file using nlohmann::json
        file >> root;
        file.close();
    }


    // Load objpoints from JSON
    for (const auto& obj : root["obj_points"]) {
        std::vector<cv::Point3f> objp;
        for (const auto& point : obj) {
            objp.emplace_back(point[0].get<float>(), point[1].get<float>(), point[2].get<float>());
        }
        objpoints.push_back(objp);
    }

    // Load imgpoints from JSON
    for (const auto& img : root["img_points"]) {
        std::vector<cv::Point2f> imgp;
        for (const auto& point : img) {
            imgp.emplace_back(point[0].get<float>(), point[1].get<float>());
        }
        imgpoints.push_back(imgp);
    }

    std::cout << "Loaded " << objpoints.size() << " object points and " << imgpoints.size() << " image points." << std::endl;

    // Initialize camera matrix and distortion coefficients
    auto K = cv::Matx33d::eye();
    auto D = cv::Vec4d::zeros();
    std::vector<cv::Mat> rvecs, tvecs;

    // Perform fisheye calibration
    cv::fisheye::calibrate(objpoints, imgpoints, cv::Size(640, 480), K, D, rvecs, tvecs,
                        cv::fisheye::CALIB_FIX_SKEW | cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC,
                        cv::TermCriteria(3, 20, 1e-6));

    // Output camera matrix and distortion coefficients
    std::cout << "Camera Matrix (K):\n" << K << std::endl;
    std::cout << "Distortion Coefficients (D):\n" << D << std::endl;

    return 0;
}
