#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Camera.h"
Camera::Camera() {
	int k = 1;
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

	this->numFeeds = captures.size();

	StoreWindowValues();
}

void Camera::StoreWindowValues() {
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
}

void Camera::TrackBall() {
	for (int i = 0; i < numFeeds; i++) {
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

	DetectCircles();
	DisplayFrames();
}

void Camera::DetectCircles() {
	for (int i = 0; i < numFeeds; i++) {
		std::vector<cv::Vec3f> circles;

		cv::HoughCircles(thresholded_gray[i], circles, CV_HOUGH_GRADIENT, 2,
			thresholded_gray[i].rows / 4, 125, 90, 10, 100);

		for (int j = 0; j < circles.size(); j++)
		{
			cv::Point center(cvRound(circles[j][0]), cvRound(circles[j][1]));
			int radius = cvRound(circles[j][2]);
			//Draw circle in the center of the ball indicating origin
			circle(frames[i], center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

			//Draw circle around ball
			circle(frames[i], center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		}
	}
}

void Camera::DisplayFrames() {
	for (int i = 0; i < numFeeds; i++) {
		std::string temporary = "Camera: " + std::to_string(i);
		const char* windowName = temporary.c_str();
		cv::imshow(windowName, frames[i]);
	}
}

void Camera::ReleaseCaptures() {
	for (int i = 0; i < numFeeds; i++) captures[i].release();
}

double Camera::CalculateDistance() {
	return distance;
}

double Camera::CalculateAngle() {
	return angle;
}

int Camera::GetNumFeeds() {
	return numFeeds;
}