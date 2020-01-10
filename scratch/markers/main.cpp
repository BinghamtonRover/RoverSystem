#include <iostream>
#include <cstdio>

#include <math.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main() {
    cv::VideoCapture cap; 

    if (!cap.open(0)) {
        fprintf(stderr, "[!] Failed to open camera!\n");
        return 1;
    }

    cv::aruco::Dictionary dictionary;

    dictionary.markerSize = 7;
    // maximum number of bit corrections
    dictionary.maxCorrectionBits = 4;

    unsigned char marker_0_data[] = {
        0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 0, 1, 1, 0,
        0, 1, 1, 0, 1, 1, 0,
        0, 1, 0, 1, 0, 1, 0,
        0, 1, 1, 1, 1, 1, 0,
        0, 1, 1, 1, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0,
    };
    auto marker_0 = cv::Mat(7, 7, CV_8UC1, marker_0_data);
    
    cv::Mat markerCompressed = cv::aruco::Dictionary::getByteListFromBits(marker_0);
    // add the marker as a new row
    dictionary.bytesList.push_back(markerCompressed);

    cv::Ptr<cv::aruco::Dictionary> dictionary_ptr = cv::Ptr<cv::aruco::Dictionary>(&dictionary);

    cv::FileStorage calib_storage("calibration.yml", cv::FileStorage::READ);
    cv::Mat camera_mat, dist_mat;
    calib_storage["camera_matrix"] >> camera_mat;
    calib_storage["distortion_coefficients"] >> dist_mat;
    
    while (true) {
        cv::Mat frame;
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        cap >> frame;

        //cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        //cv::threshold(frame, frame, 127, 255, cv::THRESH_BINARY);

        // Detect
        cv::aruco::detectMarkers(frame, dictionary_ptr, marker_corners, marker_ids);

        if (marker_ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, .2, camera_mat, dist_mat, rvecs, tvecs);
            for (int i = 0; i < rvecs.size(); ++i) {
                auto rvec = rvecs[i];
                auto tvec = tvecs[i];
                cv::aruco::drawAxis(frame, camera_mat, dist_mat, rvec, tvec, 0.1);

                auto dist = sqrt(tvec[0]*tvec[0] + tvec[1]*tvec[1] + tvec[2]*tvec[2]);
                printf("Distance: %f\n", dist);
            }
        }

        cv::imshow("image", frame);
        if (cv::waitKey(10) == 27) {
            break;
        }
    }
}
