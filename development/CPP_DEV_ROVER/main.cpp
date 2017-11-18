#include <iostream>
#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char ** argv)
{
	/*int height, width, step;

	cv::VideoCapture capture(0);
	cv::VideoCapture desktopCam(1);

	if (!desktopCam.isOpened()) {
		fprintf(stderr, "Error: laptop cam, could not be opened");
		return -2;
	}

	if (!capture.isOpened()) {
		fprintf(stderr, "Error: camera can't be opened");
		system("Pause");
		return -1;
	}

	cv::Mat frame;
	capture.read(frame);

	cv::Mat frameDesk;
	desktopCam.read(frameDesk);

	cvNamedWindow("Camera", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("DesktopCam", CV_WINDOW_AUTOSIZE);

	height = frame.rows;
	width = frame.cols;
	step = frame.step;

	int heightDesk = frameDesk.rows;
	int widthDesk = frameDesk.cols;
	int stepDesk = frameDesk.step;

	cv::Size size(width, height);

	cv::Mat thresholded_gray(size, IPL_DEPTH_8U, 1); // final thresholded image

	cv::Size sizeDesk(widthDesk, heightDesk);
	cv::Mat thresholded_gray_desk(sizeDesk, IPL_DEPTH_8U, 1);


	while (1)
	{
		// Get one frame
		bool val = capture.read(frame);
		bool valDesk = desktopCam.read(frameDesk);

		if (!valDesk)
		{
			fprintf(stderr, "ERROR: cannot capture image on desktop cam\n");
			getchar();
			break;
		}

		if (!val)
		{
			fprintf(stderr, "ERROR: cannot capture image\n");
			getchar();
			break;
		}

		//Smooth image to reduce noise
		GaussianBlur(frame, thresholded_gray, cv::Size(9, 9), 3, 3, cv::BORDER_DEFAULT);
		GaussianBlur(frameDesk, thresholded_gray_desk, cv::Size(9, 9), 3, 3, cv::BORDER_DEFAULT);

		cvtColor(thresholded_gray, thresholded_gray, CV_BGR2GRAY);
		cvtColor(thresholded_gray_desk, thresholded_gray_desk, CV_BGR2GRAY);

		//hough transform to detect circle
		std::vector<cv::Vec3f> circles;

		std::vector<cv::Vec3f> circles_desk;

		cv::HoughCircles(thresholded_gray, circles, CV_HOUGH_GRADIENT, 2,
			thresholded_gray.rows / 4, 125, 90, 10, 100);

		cv::HoughCircles(thresholded_gray_desk, circles_desk, CV_HOUGH_GRADIENT, 2,
			thresholded_gray_desk.rows / 4, 125, 90, 10, 100);

		for (int i = 0; i < circles.size(); i++)
		{
			cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
			int radius = cvRound(circles[i][2]);
			//Draw circle in the center of the ball indicating origin
			circle(frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

			//Draw circle around ball
			circle(frame, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		}

		for (int i = 0; i < circles_desk.size(); i++)
		{
			cv::Point center(cvRound(circles_desk[i][0]), cvRound(circles_desk[i][1]));
			int radius = cvRound(circles_desk[i][2]);
			//Draw circle in the center of the ball indicating origin
			circle(frameDesk, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);

			//Draw circle around ball
			circle(frameDesk, center, radius, cv::Scalar(0, 0, 255), 3, 8, 0);
		}

		cv::imshow("Camera", frame);
		cv::imshow("DesktopCam", frameDesk);


		//If ESC key pressed, close video feed
		if ((cvWaitKey(10) & 255) == 27) break;
	}

	// Release the capture device
	capture.release();
	desktopCam.release();
	cv::destroyWindow("mywindow");
	return 0;*/
	std::vector<int> height, width, step;
	std::vector<cv::Size> sizes;

	int k = 1;
	std::vector<cv::VideoCapture> captures;
	captures.push_back(cv::VideoCapture(0));
	while (captures[k-1].isOpened()) {
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

	for(int i = 0; i < captures.size(); i++) {
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

			std::string temporary = "Camera: " + std::to_string(i);
			const char* windowName = temporary.c_str();

			cv::imshow(windowName, frames[i]);
		}

		//If ESC key pressed, close video feed
		if ((cvWaitKey(10) & 255) == 27) break;
	}

	// Release the capture device
	for (int i = 0; i < numVideoFeeds; i++) captures[i].release();
	cv::destroyWindow("mywindow");
	return 0;

}