#include <opencv\cv.h>
#include <opencv\cxcore.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class Camera {
private:
	double distance;
	double angle;
	int numFeeds;
	std::vector<cv::VideoCapture> captures;
	std::vector<cv::Mat> frames, thresholded_gray;
	std::vector<int> height, width, step;
	std::vector<cv::Size> sizes;
	void StoreWindowValues();
	void DetectCircles();
	void DisplayFrames();

public:
	Camera();
	double CalculateDistance();
	double CalculateAngle();
	int GetNumFeeds();
	void TrackBall();
	void ReleaseCaptures();
};