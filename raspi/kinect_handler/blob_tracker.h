#include <Eigen/Dense>
#include <time.h>
#include <opencv2/opencv.hpp>

class BlobTracker {
	public:
		BlobTracker(int scale);
		Eigen::Vector2d predict();
		void observe(Eigen::Vector2d point);
		bool detect_leader(const cv::Mat& rgb, cv::Mat& mask, cv::Point* centroid);
	private:
		Eigen::Matrix<double,4,1> state;
		Eigen::Matrix<double,2,4> H;
		Eigen::Matrix<double,4,4> P;
		Eigen::Matrix<double,4,4> Q;
		Eigen::Matrix<double,2,2> R;
		int scale;
		bool initialized;
		double previous_time;
		Eigen::Matrix<double, 4, 4> makeF(double dt);
};
