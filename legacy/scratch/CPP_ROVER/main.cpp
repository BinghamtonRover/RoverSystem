#include <iostream>
#include <stdio.h>
#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Camera.h"

double WIDTH = 6.86; //Centimeters
double FOCAL = 700; //Dummy for now
double RESOLUTION = 1920 * 1080;

int main(int argc, char ** argv) {
	std::vector<int> height, width, step;
	std::vector<cv::Size> sizes;

	int k = 1;
	std::vector<cv::VideoCapture> captures;
	captures.push_back(cv::VideoCapture(0));

	while (captures[k - 1].isOpened()) {
		cv::VideoCapture cap = cv::VideoCapture(k);
		if (cap.isOpened()) {
			captures.push_back(cap);
		}
		else {
			break;
		}
		k++;
	}

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

	while (1)
	{
		// Get one frame
		for (int i = 0; i < numVideoFeeds; i++) {
			bool val = captures[i].read(frames[i]);
			if (!val) {
				fprintf(stderr, "ERROR: cannot capture image on camera\n");
				getchar();
				break;
			}
			//Smooth image to reduce noise
			GaussianBlur(frames[i], thresholded_gray[i], cv::Size(9, 9), 3, 3, cv::BORDER_DEFAULT);

			//Convert to grayscale to do houghCircle transformation
			cvtColor(thresholded_gray[i], thresholded_gray[i], CV_BGR2GRAY);
		}

		for (int i = 0; i < numVideoFeeds; i++) {
			std::vector<cv::Vec3f> circles;
			cv::HoughCircles(thresholded_gray[i], circles, CV_HOUGH_GRADIENT, 2,
				thresholded_gray[i].rows / 4, 100, 100, 20, 200);

			for (int j = 0; j < circles.size(); j++)
			{
				cv::Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
				int radius = cvRound(circles[j][2]);

				//TODO Need to initialize focal length
				int pixelWidth = radius * 2;
				int distance = (WIDTH * FOCAL) / pixelWidth;
				printf("Distance (centimeters): %d\n", distance);

				//TODO ANGLE CALCULATIONS

				//Draw circle in the center of the ball indicating origin
				circle(frames[i], center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

				//Draw circle around ball
				circle(frames[i], center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
			}

			std::string temporary = "Camera: " + std::to_string(i);
			const char* windowName = temporary.c_str();

			cv::imshow(windowName, frames[i]);
		}

		//If ESC key pressed, close video feed
		if ((cvWaitKey(10) & 255) == 27) break;
	}

	// Release the capture device
	for (int i = 0; i < numVideoFeeds; i++) captures[i].release();

	return 0;
}