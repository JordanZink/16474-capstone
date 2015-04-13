#include "crowd_detector.h"
#include "blob_tracker.h"
#include "my_freenect_device.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <errno.h>

#define MIN_FACES 1
#define MAX_DEPTH 123
#define DIS 1
#define SCALE 8
#define WIDTH 640
#define HEIGHT 480
#define SWIDTH (WIDTH/SCALE)
#define SHEIGHT (HEIGHT/SCALE)
#define DWIDTH 640
#define DHEIGHT 480

#define FOLLOW_MODE 0
#define CROWD_MODE 1

#define RESCALE_X(xs) (((xs - (SWIDTH/2))*SCALE)+(WIDTH/2))
#define RESCALE_Y(ys) (((ys - (SHEIGHT/2))*SCALE)+(HEIGHT/2))
#define DISPLAY_TO_IM_X(xs) (((xs - (DWIDTH/2))/SCALE)+(SWIDTH/2))
#define DISPLAY_TO_IM_Y(ys) (((ys - (DHEIGHT/2))/SCALE)+(SHEIGHT/2))

BlobTracker tracker(SCALE);
cv::Mat rgbMat,depthMat;

static int Hmin,Smin,Vmin;
static int Hmax,Smax,Vmax;
static int mode = FOLLOW_MODE;

void update_trackbars() {
    cv::setTrackbarPos("Hmin", "display", Hmin);
    cv::setTrackbarPos("Smin", "display", Smin);
    cv::setTrackbarPos("Vmin", "display", Vmin);
    cv::setTrackbarPos("Hmax", "display", Hmax);
    cv::setTrackbarPos("Smax", "display", Smax);
    cv::setTrackbarPos("Vmax", "display", Vmax);
}

void on_trackbar(int q, void* )
{
    std::cout << q << std::endl;
    std::cout << Hmin << "," << Hmax << "," << Smin << "," << Smax << "," << Vmin << "," << Vmax << std::endl;
    tracker.set_bounds(Hmin,Hmax,Smin,Smax,Vmin,Vmax);
    update_trackbars();
}

void mouse_callback(int event, int x, int y, int flags, void* userdata) {
    if (event == cv::EVENT_LBUTTONDOWN) {
	//std::cout << x << "," << y << std::endl;
	cv::Vec3b col = rgbMat.at<cv::Vec3b>(DISPLAY_TO_IM_X(x),DISPLAY_TO_IM_Y(y));
	//std::cout << (int)col.val[0] << "," << (int)col.val[1] << "," << (int)col.val[2] << std::endl;
	if (col.val[0] < Hmin) Hmin = col.val[0];
	if (col.val[0] > Hmax) Hmax = col.val[0];
	if (col.val[1] < Smin) Smin = col.val[1];
	if (col.val[1] > Smax) Smax = col.val[1];
	if (col.val[2] < Vmin) Vmin = col.val[2];
	if (col.val[2] > Vmax) Vmax = col.val[2];
	update_trackbars();
	tracker.set_bounds(Hmin,Hmax,Smin,Smax,Vmin,Vmax);
    }
}

int main(int argc, char **argv) {
    bool die = false;
    cv::Mat rgbMatb(cv::Size(WIDTH, HEIGHT), CV_8UC3);
    cv::Mat depthMatb(cv::Size(WIDTH, HEIGHT),CV_16UC1);

    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

#if DIS
    cv::namedWindow("display",CV_WINDOW_AUTOSIZE);
    cv::namedWindow("mask",CV_WINDOW_AUTOSIZE);
    Hmin = 81;
    Smin = 180;
    Vmin = 196;
    Hmax = 255;
    Smax = 217;
    Vmax = 255;
    tracker.set_bounds(Hmin,Hmax,Smin,Smax,Vmin,Vmax);
    cv::createTrackbar("Hmin", "display", &Hmin, 255, on_trackbar);
    cv::createTrackbar("Smin", "display", &Smin, 255, on_trackbar);
    cv::createTrackbar("Vmin", "display", &Vmin, 255, on_trackbar);
    cv::createTrackbar("Hmax", "display", &Hmax, 255, on_trackbar);
    cv::createTrackbar("Smax", "display", &Smax, 255, on_trackbar);
    cv::createTrackbar("Vmax", "display", &Vmax, 255, on_trackbar);
    cv::setMouseCallback("display", mouse_callback, NULL);
#endif
    device.startVideo();
    device.startDepth();
    cv::Mat K = cv::Mat::zeros(3, 3, CV_64F);
    K.at<double>(0,0) = 517.05;
    K.at<double>(1,1) = 517.02;
    K.at<double>(0,2) = 324.18;
    K.at<double>(1,2) = 262.17;
    K.at<double>(2,2) = 1.0;
    cv::Mat K1 = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat K2 = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat K3 = cv::Mat::zeros(3, 3, CV_64F);
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
    cv::Mat scale_mat = cv::Mat::zeros(3, 3, CV_64F);
    scale_mat = K3*K2*K1;

    CrowdDetector detector(MIN_FACES,MAX_DEPTH,SCALE);

    std::string command;

    while (std::getline(std::cin, command)) {
	if (command[0] == 'F') {
	    mode = FOLLOW_MODE;
	} else if (command[0] == 'C') {
	    mode = CROWD_MODE;
	} else {
	    std::cout << "E: bad command" << std::endl;
	}
	device.getVideo(rgbMatb);
	cv::resize(rgbMatb,rgbMat, cv::Size(SWIDTH,SHEIGHT));
	cv::cvtColor(rgbMat, rgbMat, CV_RGB2HSV);
	device.getDepth(depthMatb);
#if DIS
	cv::Mat display;
	rgbMat.copyTo(display);
#endif
	cv::Mat mask;
	if (mode == FOLLOW_MODE) {
	    cv::Point blob_centroid;
	    bool predicted;
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
	    cv::Mat leader_direction(3,1,CV_64F);
	    leader_direction.at<double>(0,0) = blob_centroid.x;
	    leader_direction.at<double>(1,0) = blob_centroid.y;
	    leader_direction.at<double>(2,0) = 1.0;
	    cv::Mat lead_vec = K.inv()*scale_mat*leader_direction;
	    lead_vec = lead_vec/cv::norm(lead_vec);
	    if (predicted) {
		std::cout << "P: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << "," << depthMatb.at<unsigned short>(RESCALE_Y(blob_centroid.y), RESCALE_X(blob_centroid.x))<< std::endl;
	    } else {
		std::cout << "O: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << "," << depthMatb.at<unsigned short>(RESCALE_Y(blob_centroid.y), RESCALE_X(blob_centroid.x))<< std::endl;
	    }
	} else if (mode == CROWD_MODE) {
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
		cv::Mat cent_vec = K.inv()*scale_mat*centroid_direction;
		cent_vec = cent_vec/cv::norm(cent_vec);
		std::cout << "C: " << cent_vec.at<double>(0,0) << "," << cent_vec.at<double>(1,0) << "," << cent_vec.at<double>(2,0) << "," << depthMatb.at<unsigned short>(RESCALE_Y(crowd_centroid.y), RESCALE_X(crowd_centroid.x)) << std::endl;
	    }
	}
#if DIS
	cv::Mat im_display;
	cv::Mat im_mask;
	cv::resize(display,im_display,cv::Size(DWIDTH,DHEIGHT));
	cv::resize(mask,im_mask,cv::Size(DWIDTH,DHEIGHT));
	cv::imshow("display", im_display);
	cv::imshow("mask", im_mask);
#endif
	char k = cv::waitKey(5);
    }

    device.stopVideo();
    device.stopDepth();
    return 0;
}

