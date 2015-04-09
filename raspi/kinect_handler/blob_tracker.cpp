#include "blob_tracker.h"
#include "timing.h"

BlobTracker::BlobTracker(int scale_) : scale(scale_) {
	// Observation
	H << 1, 0, 0, 0,
	  0, 1, 0, 0;
	// Initial Covariance
	P << 0.1, 0, 0, 0,
	  0, 0.1, 0, 0,
	  0, 0, 0.01, 0,
	  0, 0, 0, 0.01;
	// Process Noise
	Q << 0.1, 0, 0, 0,
	  0, 0.1, 0, 0,
	  0, 0, 0.01, 0,
	  0, 0, 0, 0.01;
	R << 5.0, 0,
	  0, 5.0;
	initialized = false;
}
Eigen::Vector2d BlobTracker::predict() {
	if (!initialized) {
		return Eigen::Vector2d(-1,-1);
	} else {
		timespec time1;
		gettime(&time1);
		double time = time1.tv_sec + ((double)time1.tv_nsec)/1.0e9;
		double dt = time - previous_time;
		Eigen::Matrix4d thisF = makeF(dt);
		Eigen::Vector4d pred_x = thisF*state;
		return pred_x.head(2);
	}
}
void BlobTracker::observe(Eigen::Vector2d point) {
	timespec time1;
	gettime(&time1);
	double time = time1.tv_sec + ((double)time1.tv_nsec)/1.0e9;
	if (!initialized) {
		initialized = true;
		previous_time = time;
		state(0) = point.x();
		state(1) = point.y();
		state(2) = 0;
		state(3) = 0;
	} else {
		double dt = time - previous_time;
		Eigen::Matrix4d thisF = makeF(dt);
		Eigen::Vector4d pred_x = thisF*state;
		Eigen::Matrix4d pred_P = thisF*P*thisF.transpose() + Q;

		Eigen::Vector2d predicted_pos = H*pred_x;
		Eigen::Vector2d err = point - predicted_pos;
		Eigen::Matrix2d S = H*pred_P*H.transpose() + R;
		Eigen::Matrix<double, 4, 2> K = pred_P*H.transpose()*S.inverse();
		state = pred_x + K*err;
		P = (Eigen::Matrix4d::Identity() - K*H)*pred_P;

		previous_time = time;
	}
}
bool BlobTracker::detect_leader(const cv::Mat& rgb, cv::Mat& mm, cv::Point* centroid) {
	cv::Mat rr;
	cv::cvtColor(rgb, rr, CV_RGB2HSV);
	cv::blur(rr, rr, cv::Size(11,11));
	mm = cv::Mat(rgb.rows, rgb.cols, CV_8U);
	cv::inRange(rr,cv::Scalar(37, 105,0), cv::Scalar(45,140,255),mm);
	cv::Moments m=cv::moments(mm, true);
	double area = m.m00;
	double c_x = m.m10/m.m00;
	double c_y = m.m01/m.m00;
	if (area >= 1000/(scale*scale)) {
	    centroid->x = c_x;
	    centroid->y = c_y;
	    return true;
	} else {
	    centroid->x = 320;
	    centroid->y = 240;
	    return false;
	}
}
Eigen::Matrix<double, 4, 4> BlobTracker::makeF(double dt) {
	Eigen::Matrix<double,4,4> F;
	F << 1, 0, dt, 0,
	  0, 1, 0, dt,
	  0, 0, 1, 0,
	  0, 0, 0, 1;
	return F;
}
