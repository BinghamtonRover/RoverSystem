#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Camera.h"

using namespace std;
using namespace cv;

double WIDTH = 6.86; //Centimeters
double FOCAL = 700; //Dummy for now
double RESOLUTION = 1920 * 1080;

int thresh = 100;

int main(int argc, char ** argv) {
    std::vector<int> height, width, step;
    std::vector<cv::Size> sizes;

    int k = 1;
    std::vector<cv::VideoCapture> captures;
    captures.push_back(cv::VideoCapture(0));

    captures[0].set(CV_CAP_PROP_FRAME_WIDTH , 352);
    captures[0].set(CV_CAP_PROP_FRAME_HEIGHT , 288);
    while (captures[k - 1].isOpened()) {
        cv::VideoCapture cap = cv::VideoCapture(k);
        if (cap.isOpened()) {
            captures[k].set(CV_CAP_PROP_FRAME_WIDTH , 352);
            captures[k].set(CV_CAP_PROP_FRAME_HEIGHT , 288);
            captures.push_back(cap);
        }
        else {
            break;
        }
        k++;
    }

    cvNamedWindow("Mask", CV_WINDOW_AUTOSIZE);

    int numVideoFeeds = captures.size();
    std::vector<cv::Mat> frames;
    std::vector<cv::Mat> thresholded_gray;

    for (int i = 0; i < captures.size(); i++) {
        std::string temporary = "Camera: " + std::to_string(i);
        const char* windowName = temporary.c_str();
        cv::Mat frame;
        captures[i].read(frame);
        frames.push_back(frame);
        cvNamedWindow(windowName, CV_WINDOW_AUTOSIZE);
        height.push_back(frame.rows);
        width.push_back(frame.cols);
        step.push_back(frame.step);
        sizes.push_back(cv::Size(width[i], height[i]));

        cv::Mat temp(sizes[i], IPL_DEPTH_8U, 1);
        thresholded_gray.push_back(temp);
    }

    while (1) {
        // Get one frame
        for (int i = 0; i < numVideoFeeds; i++) {
            bool val = captures[i].read(frames[i]);
            if (!val) {
                fprintf(stderr, "ERROR: cannot capture image on camera\n");
                getchar();
                break;
            }
        }

        for (int i = 0; i < numVideoFeeds; i++) {
            /// HSV mask and contours container
            cv::Mat mask;
            std::vector< std::vector< cv::Point>> contours;

            /*---Pre-processing Image Transformations---*/
            /// Blur to allow for faster processing ? Not sure if neccesary ! Remove grayscale processing
            GaussianBlur(frames[i], thresholded_gray[i], cv::Size(9, 9), 3, 3, cv::BORDER_DEFAULT);
            /// Change to HSV
            cvtColor(frames[i], mask , CV_BGR2HSV);
            /// Apply color range for HSV
            cv::inRange(mask, cv::Scalar(29, 51, 6), cv::Scalar(64, 255, 255), mask);
            /// Erode and dilate to remove unncessary shapes
            cv::erode(mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue());
            cv::dilate(mask, mask, Mat(), Point(-1, -1), 2, BORDER_CONSTANT, morphologyDefaultBorderValue());

            /*---Find contours in mask---*/
            cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
            //If contours found, measure and draw them
            if(contours.size() > 0){
                /// Sort to find largest contour
                sort(contours.begin(), contours.end(), [&contours](vector<Point> lhs, vector<Point> rhs){
                    return contourArea(lhs) > contourArea(rhs);
                });

                /// Bounding circle for largest contour
                Point2f contour_point;
                float contour_radius;
                minEnclosingCircle( (Mat)contours[0], contour_point, contour_radius);
                if(contour_radius > 15){
                    //Draw biggest bounding circle on original image
                    drawContours(frames[i], contours, 0, Scalar(255, 0, 0), 1, 8, vector<Vec4i>(), 0, Point());
                    circle(frames[i], contour_point, (int)contour_radius, Scalar(0, 0, 255), 2, 8 ,0);
                }
            }

            std::string temporary = "Camera: " + std::to_string(i);
            const char* windowName = temporary.c_str();

            cv::imshow(windowName, frames[i]);
            cv::imshow("Mask", mask);

        }

        //If ESC key pressed, close video feed
        if ((cvWaitKey(10) & 255) == 27) break;
    }

    // Release the capture device
    for (int i = 0; i < numVideoFeeds; i++) captures[i].release();

    return 0;
}
