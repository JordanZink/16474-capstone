#include "crowd_detector.h"
#include <cstdio>

CrowdDetector::CrowdDetector(int min_faces_, int max_depth_, int scale_) : min_faces(min_faces_), max_depth(max_depth_), scale(scale_) {
	if(!front_face_cascade.load("haarcascade_frontalface_alt.xml")) {
		printf("--(!)Error loading\n");
	}
	if(!profile_face_cascade.load("haarcascade_profileface.xml")) {
		printf("--(!)Error loading\n");
	}

}

bool CrowdDetector::detectCrowd(const cv::Mat& depth, const cv::Mat rgb, cv::Point* centroid, std::vector<cv::Point>& ff) {
	cv::Mat depthf;
	depth.convertTo(depthf, CV_8UC1, 255.0/2048.0);
	cv::Mat mask = depthf <= max_depth;
	cv::Mat fg;
	rgb.copyTo(fg, mask);
	cv::Mat fg_gray;
	cv::cvtColor(fg, fg_gray, CV_BGR2GRAY);
	std::vector<cv::Rect> faces, profiles;
	front_face_cascade.detectMultiScale(fg_gray, faces, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(20/scale, 20/scale));
	profile_face_cascade.detectMultiScale(fg_gray, profiles, 1.1, 2, CV_HAAR_SCALE_IMAGE, cv::Size(20/scale, 20/scale));
	if (faces.size() + profiles.size() >= min_faces) {
		cv::Point centroid_sum(0,0);
		for (int i = 0; i < faces.size(); i++) {
			ff.push_back(cv::Point(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5));
			centroid_sum += cv::Point(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
		}
		for (int i = 0; i < profiles.size(); i++) {
			ff.push_back(cv::Point(profiles[i].x + profiles[i].width*0.5, profiles[i].y + profiles[i].height*0.5));
			centroid_sum += cv::Point(profiles[i].x + profiles[i].width*0.5, profiles[i].y + profiles[i].height*0.5);
		}
		double ct = (double)(faces.size() + profiles.size());
		centroid->x = centroid_sum.x/ct;
		centroid->y = centroid_sum.y/ct;
		return true;
	} else {
		return false;
	}
}
