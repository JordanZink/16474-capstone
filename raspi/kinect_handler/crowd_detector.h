#include <opencv2/opencv.hpp>

class CrowdDetector {
	public:
		CrowdDetector(int min_faces, int max_depth, int scale);
		bool detectCrowd(const cv::Mat rgb, cv::Point* centroid, std::vector<cv::Point>& ff);
	private:
		int min_faces;
		int max_depth;
		int scale;
		cv::CascadeClassifier front_face_cascade;
		cv::CascadeClassifier profile_face_cascade;

};
