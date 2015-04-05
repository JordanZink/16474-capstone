#include "crowd_detector.h"
#include "blob_tracker.h"
#include "my_freenect_device.h"
//#include "libfreenect/libfreenect.hpp"
#include <iostream>
#include <vector>
//#include <cmath>
#include <opencv2/opencv.hpp>
#include <cstdio>

#define MIN_FACES 1
#define MAX_DEPTH 123
#define DIS 1
#define SCALE 2
#define WIDTH 640
#define HEIGHT 480
#define SWIDTH (WIDTH/SCALE)
#define SHEIGHT (HEIGHT/SCALE)
#define DWIDTH 1280
#define DHEIGHT 720

int main(int argc, char **argv) {
	bool die = false;
	cv::Mat rgbMatb(cv::Size(WIDTH, HEIGHT), CV_8UC3);
	cv::Mat depthMatb(cv::Size(WIDTH, HEIGHT),CV_16UC1);

	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

#if DIS
	cv::namedWindow("display",CV_WINDOW_AUTOSIZE);
	cv::namedWindow("mask",CV_WINDOW_AUTOSIZE);
#endif
	device.startVideo();
	device.startDepth();
	cv::Mat Ki(3,3,CV_64F);
	Ki.at<double>(0,0) = 517.05;
	Ki.at<double>(1,1) = 517.02;
	Ki.at<double>(0,2) = 324.18;
	Ki.at<double>(1,2) = 262.17;
	Ki.at<double>(2,2) = 1.0;
	cv::Mat K1(3,3,CV_64F);
	cv::Mat K2(3,3,CV_64F);
	cv::Mat K3(3,3,CV_64F);
	K1.at<double>(0,0) = 1;
	K1.at<double>(1,1) = 1;
	K1.at<double>(0,2) = -(SWIDTH/2);
	K1.at<double>(1,2) = -(SHEIGHT/2);
	K1.at<double>(2,2) = 1.0;
	K2.at<double>(0,0) = SCALE;
	K2.at<double>(1,1) = SCALE;
	K2.at<double>(2,2) = 1.0;
	K3.at<double>(0,0) = 1;
	K3.at<double>(1,1) = 1;
	K3.at<double>(0,2) = 320;
	K3.at<double>(1,2) = 240;
	K3.at<double>(2,2) = 1.0;
	cv::Mat K(3,3,CV_64F);
	K = Ki*K3*K2*K1;



	BlobTracker tracker(SCALE);
	CrowdDetector detector(MIN_FACES,MAX_DEPTH,SCALE);
	cv::Mat rgbMat,depthMat;

	while (!die) {
		device.getVideo(rgbMatb);
		cv::resize(rgbMatb,rgbMat, cv::Size(SWIDTH,SHEIGHT));
		device.getDepth(depthMatb);
		cv::resize(depthMatb,depthMat, cv::Size(SWIDTH,SHEIGHT));
#if DIS
		cv::Mat display;
		rgbMat.copyTo(display);
#endif

		cv::Point blob_centroid;
		bool predicted;
		cv::Mat mask;
		if (tracker.detect_leader(rgbMat,mask,&blob_centroid)) {
			Eigen::Vector2d p(blob_centroid.x, blob_centroid.y);
			tracker.observe(p);
#if DIS
			cv::circle(display, cv::Point(p.x(), p.y()), 20/SCALE, CV_RGB(0,0,255));
#endif
			predicted = false;
		} else {
			Eigen::Vector2d pred_loc = tracker.predict();
#if DIS
			cv::circle(rgbMat, cv::Point(pred_loc.x(), pred_loc.y()), 20/SCALE, CV_RGB(0,255,0));
#endif
			predicted = true;
		}
		cv::Point crowd_centroid;
		std::vector<cv::Point> faces;
		if (detector.detectCrowd(depthMat,rgbMat,&crowd_centroid, faces)) {
#if DIS
			for (int i = 0; i < faces.size(); i++) {
				cv::circle(display, faces[i], 20/SCALE, CV_RGB(255,255,0));
			}
			cv::circle(display, crowd_centroid, 20/SCALE, CV_RGB(255,0,0));
#endif
			cv::Mat centroid_direction(3,1,CV_64F);
			centroid_direction.at<double>(0,0) = crowd_centroid.x;
			centroid_direction.at<double>(1,0) = crowd_centroid.y;
			centroid_direction.at<double>(2,0) = 1.0;
			cv::Mat cent_vec = K.inv()*centroid_direction;
			cent_vec = cent_vec/cv::norm(cent_vec);
			std::cout << "C: " << cent_vec.at<double>(0,0) << "," << cent_vec.at<double>(1,0) << "," << cent_vec.at<double>(2,0) << std::endl;
		}

#if DIS
		cv::Mat im_display;
		cv::Mat im_mask;
		cv::resize(display,im_display,cv::Size(DWIDTH,DHEIGHT));
		cv::resize(mask,im_mask,cv::Size(DWIDTH,DHEIGHT));
		cv::imshow("display", im_display);
		cv::imshow("mask", im_mask);
#endif

		cv::Mat leader_direction(3,1,CV_64F);
		leader_direction.at<double>(0,0) = blob_centroid.x;
		leader_direction.at<double>(1,0) = blob_centroid.y;
		leader_direction.at<double>(2,0) = 1.0;
		cv::Mat lead_vec = K.inv()*leader_direction;
		lead_vec = lead_vec/cv::norm(lead_vec);
		if (predicted) {
			std::cout << "P: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << std::endl;
		} else {
			std::cout << "O: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << std::endl;
		}

		char k = cv::waitKey(5);
	}

	device.stopVideo();
	device.stopDepth();
	return 0;
}

