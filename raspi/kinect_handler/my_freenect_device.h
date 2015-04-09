#include "libfreenect/libfreenect.hpp"
#include <opencv2/opencv.hpp>
#include "blob_tracker.h"
#include "crowd_detector.h"

class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int index);
		void VideoCallback(void* _rgb, uint32_t timestamp);
		void DepthCallback(void* _depth, uint32_t timestamp);
		void start(void);
	private:
		cv::Mat depthMat;
		cv::Mat rgbMat;
		cv::Mat rgbMat_s;
		cv::Mat K;
		cv::Mat resize;
		BlobTracker* tracker;
		pthread_mutex_t leader_pos_lock;
		cv::Point leader_loc;
};
