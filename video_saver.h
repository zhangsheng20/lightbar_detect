#ifndef VIDEO_SAVER_H
#define VIDEO_SAVER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
using namespace std;
using namespace cv;



class VideoSaver{
    public:
        VideoSaver(int num = 1500);
        void write(Mat frame);
    private:
        int frameCount;
        int maxFrame;
        VideoWriter writer;
};



#endif // VIDEO_SAVER_H
