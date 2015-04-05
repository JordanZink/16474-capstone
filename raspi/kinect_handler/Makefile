CXXFLAGS=`pkg-config --cflags opencv eigen3 libusb-1.0`
LDLIBS=`pkg-config --libs opencv libusb-1.0` -lpthread -lfreenect

SRCS=kinect_handler.cpp my_freenect_device.cpp blob_tracker.cpp crowd_detector.cpp
OBJS=$(subst .cpp,.o,$(SRCS))


all: kinect_handler

kinect_handler: $(OBJS)
	g++ -O3 $(OBJS) -o kinect_handler $(LDLIBS)

clean:
	rm $(OBJS) kinect_handler
