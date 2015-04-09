#include "my_freenect_device.h"
#include <iostream>
#include <vector>
#include <cstdio>

#define SCALE 4
//#define MIN_FACES 1
//#define MAX_DEPTH 123
//#define DIS 1
#define WIDTH 640
#define HEIGHT 480
#define SWIDTH (WIDTH/SCALE)
#define SHEIGHT (HEIGHT/SCALE)

#define RESCALE_X(xs) (((xs - (SWIDTH/2))*SCALE)+(WIDTH/2))
#define RESCALE_Y(ys) (((ys - (SHEIGHT/2))*SCALE)+(HEIGHT/2))

MyFreenectDevice::MyFreenectDevice(freenect_context *_ctx, int _index)
    : Freenect::FreenectDevice(_ctx, _index),
    depthMat(cv::Size(WIDTH,HEIGHT),CV_16UC1, cv::Scalar(0)),
    rgbMat(cv::Size(WIDTH,HEIGHT), CV_8UC3, cv::Scalar(0)),
    rgbMat_s(cv::Size(SWIDTH,SHEIGHT), CV_8UC3, cv::Scalar(0)) {
	K = cv::Mat::zeros(3, 3, CV_64F);
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
	resize = cv::Mat::zeros(3, 3, CV_64F);
	resize = K3*K2*K1;
	tracker = new BlobTracker(SCALE);

	pthread_mutex_init(&leader_pos_lock, NULL);
    }

void MyFreenectDevice::start(void) {
    startVideo();
    startDepth();
}
// Do not call directly even in child
void MyFreenectDevice::VideoCallback(void* _rgb, uint32_t timestamp) {
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    rgbMat.data = rgb;
    cv::pyrDown(rgbMat,rgbMat_s, cv::Size(SWIDTH,SHEIGHT));
    cv::Point blob_centroid;
    bool predicted;
    cv::Mat mask;
    if (tracker->detect_leader(rgbMat_s, mask, &blob_centroid)) {
	Eigen::Vector2d p(blob_centroid.x, blob_centroid.y);
	tracker->observe(p);
	predicted = false;
    } else {
	Eigen::Vector2d pred_loc = tracker->predict();
	blob_centroid.x = pred_loc.x();
	blob_centroid.y = pred_loc.y();
	predicted = true;
    }
    pthread_mutex_lock(&leader_pos_lock);
    leader_loc = blob_centroid;
    pthread_mutex_unlock(&leader_pos_lock);
    cv::Mat leader_direction(3,1,CV_64F);
    leader_direction.at<double>(0,0) = blob_centroid.x;
    leader_direction.at<double>(1,0) = blob_centroid.y;
    leader_direction.at<double>(2,0) = 1.0;
    cv::Mat lead_vec = K.inv()*resize*leader_direction;
    lead_vec = lead_vec/cv::norm(lead_vec);
    if (predicted) {
	    std::cout << "P: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << std::endl;
    } else {
	    std::cout << "O: " << lead_vec.at<double>(0,0) << "," << lead_vec.at<double>(1,0) << "," << lead_vec.at<double>(2,0) << std::endl;
    }
}

// Do not call directly even in child
void MyFreenectDevice::DepthCallback(void* _depth, uint32_t timestamp) {
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    depthMat.data = (uchar*) depth;
    pthread_mutex_lock(&leader_pos_lock);
    cv::Point blob_pos = leader_loc;
    pthread_mutex_unlock(&leader_pos_lock);

    double blob_depth;
    if (blob_pos.x == 320 && blob_pos.y == 240) {
	blob_depth = 810;
    } else {
	blob_depth = depthMat.at<unsigned short>(RESCALE_Y(blob_pos.y), RESCALE_X(blob_pos.x));
    }
    std::cout << "B: " << blob_depth << std::endl;
}
