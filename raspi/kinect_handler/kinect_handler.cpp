#include "my_freenect_device.h"

int main(int argc, char **argv) {
	Freenect::Freenect freenect;
	MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);
	device.start();
	/*

#if DIS
	cv::namedWindow("display",CV_WINDOW_AUTOSIZE);
	cv::namedWindow("mask",CV_WINDOW_AUTOSIZE);
#endif
	CrowdDetector detector(MIN_FACES,MAX_DEPTH,SCALE);
	cv::Mat rgbMat,depthMat;

	while (!die) {
		device.getVideo(rgbMatb);
		//cv::resize(rgbMatb,rgbMat, cv::Size(SWIDTH,SHEIGHT));
		cv::pyrDown(rgbMatb,rgbMat, cv::Size(SWIDTH,SHEIGHT));
		device.getDepth(depthMatb);
		//cv::resize(depthMatb,depthMat, cv::Size(SWIDTH,SHEIGHT));
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
		if (detector.detectCrowd(rgbMat,&crowd_centroid, faces)) {
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
	*/
	return 0;

}

