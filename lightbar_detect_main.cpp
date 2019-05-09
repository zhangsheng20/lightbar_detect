#include <iostream>
#include <thread>
 #include <mutex>
#include <stdio.h>
#include "stdlib.h"


#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/opengl.hpp"
#include "opencv2/video.hpp"     //BackgroundSubtractor在里面


#include "video_saver.h"
#include "data_send.h"
#include "serial.h"
#include "lightbar_detect.h"

using namespace std;
using namespace cv;
#define TX2         //
//#define SERIAL_SEND //是否开启串口并发送数据
#define SHOW_FRAMES //是否显示图像
//#define VIDEO_SAVE



void DisplayFps(Mat& img);//desplay FPS
int OpenVideoStream(int camWay);
void SetCameraPara();
void CoutFps();        //

int camWay = 1; // 0: camera1 1: camera2  2: vedio


VideoCapture capture;
Serial sel;
double myVideoCaptureProperties[50];   //存储摄像头参数



//将要打开的视频路径  注意‘/’和‘\’
String video_file_name1 = "/home/sheng/桌面/大符视频/2019-5-2_22-49-2.avi";  //晃动的
String video_file_name2 = "/home/sheng/桌面/大符视频/2019-5-2_22-48-37.avi";  //不晃的

String video_file_name=video_file_name1;
extern Point2f SendYawPitchErrorCurrent;            //存储需要发送的装甲板信息
extern int  IsDetectedFlag ;                 //是否检测到装甲板信息

#define SerialPort_COM 1    //用于发送的串口的串口号

//不同的操作系统有些配置不一样
#define WINDOWS_OS
//#define LINUX_OS




//InputImage为三通道图像，将第三通道threshold
Mat mythreshold(Mat &InputImage, int threshold)
{
    Mat OutImage(400, 640, CV_8UC1);

    int channels = InputImage.channels();  //通道数
    int nRows = InputImage.rows;            //行数
    int nCols = InputImage.cols* channels;  //列数


    int i, j;
    uchar* pInputImage;
    uchar* pOutImage;
    for (i = 0; i < nRows; ++i)     //行数
    {
        pInputImage = InputImage.ptr<uchar>(i);
        pOutImage = OutImage.ptr<uchar>(i);
        for (j = 0; j < InputImage.cols; ++j)
        {
            if (pInputImage[j * 3 + 2] > threshold)
                pOutImage[j] = 255;
            else
                pOutImage[j] = 0;
        }
    }
    return OutImage;
}

VideoSaver videosaver;




int main()
{
    #ifdef SERIAL_SEND
    {//打开串口     
        if(sel.setPara(115200,8,1,'n'))
         cout<<"config success"<<endl;
        else
         cout<<"config failed"<<endl;
    }
    #endif

    Mat frame_read;
    Mat threshold_frame;             //二值化图像存贮

    OpenVideoStream(camWay);


    SetCameraPara();

    while (capture.read(frame_read)) {
        #ifdef VIDEO_SAVE
             videosaver.write(frame_read);
        #endif

        threshold_frame = mythreshold(frame_read, 200);  //图像二值化
        DrawEnclosingRexts(threshold_frame, frame_read);   //画灯条，求装甲板位置，算云台Yaw和Pitch偏角
        cout << SendYawPitchErrorCurrent.x << "   " << SendYawPitchErrorCurrent.y << endl;


        #ifdef SHOW_FRAMES
            imshow("src",frame_read);
            char c = waitKey(1);
        #endif

        #ifdef SERIAL_SEND
            SendDataToInfantry();
        #endif


        CoutFps();
    }

    return 0;
}


int OpenVideoStream(int camWay)
{


    if (camWay == 2) {
        capture.open(video_file_name); return true;
    }
    else if (camWay == 0) {
        capture.open(0); return true;
    }
    else if (camWay == 1) {
        capture.open(0); return true;
    }

    if (!capture.isOpened())
    {
        printf("can not open camera or video file\n");
        return -1;
    }



}




void CoutFps()
{
    double  fps = 0;
    double t = 0;

    static double time1 = 0.001;
    static double time2 = 0.002;
    time2 = time1;
    time1 = (double)cv::getTickCount();

    t = (time1 - time2) / cv::getTickFrequency();
    fps = 1.0 / t;

    //std::cout << t << "\n";
    cout <<"fps"<< fps << "\n";
    //std::cout << cv::getTickFrequency() << "\n\n";

}

void SetCameraPara()
{


    capture.set(CAP_PROP_FRAME_WIDTH, 640);//宽度
   capture.set(CAP_PROP_FRAME_HEIGHT, 400);//高度  分辨率设置成640*400时帧率是240
    capture.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    capture.set(CAP_PROP_AUTO_EXPOSURE, 0.25);// where 0.25 means "manual exposure, manual iris"
    capture.set(CAP_PROP_IRIS, 10);
    capture.set(CAP_PROP_EXPOSURE, -8);

    myVideoCaptureProperties[CAP_PROP_AUTO_EXPOSURE] = capture.get(CAP_PROP_AUTO_EXPOSURE);
    cout << "CAP_PROP_AUTO_EXPOSURE:" << myVideoCaptureProperties[CAP_PROP_AUTO_EXPOSURE] << endl;

    myVideoCaptureProperties[CAP_PROP_IRIS] = capture.get(CAP_PROP_IRIS);
    cout << "CAP_PROP_IRIS:" << myVideoCaptureProperties[CAP_PROP_IRIS] << endl;

    myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] = capture.get(CAP_PROP_FRAME_WIDTH);
    cout << "FRAME_WIDTH:" << myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] << endl;

    myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] = capture.get(CAP_PROP_FRAME_HEIGHT);
    cout << "FRAME_HEIGHT:" << myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] << endl;

    myVideoCaptureProperties[CAP_PROP_FPS] = capture.get(CAP_PROP_FPS);
    cout << "CAP_PROP_FPS:" << myVideoCaptureProperties[CAP_PROP_FPS] << endl;


    myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] = capture.get(CAP_PROP_BRIGHTNESS); //亮度
    cout << "CAP_PROP_BRIGHTNESS:" << myVideoCaptureProperties[CAP_PROP_BRIGHTNESS] << endl;


    myVideoCaptureProperties[CAP_PROP_CONTRAST] = capture.get(CAP_PROP_CONTRAST); //对比度
    //cout << "CAP_PROP_CONTRAST:" << myVideoCaptureProperties[CAP_PROP_CONTRAST] << endl;


    myVideoCaptureProperties[CAP_PROP_SATURATION] = capture.get(CAP_PROP_SATURATION); //饱和度
    myVideoCaptureProperties[CAP_PROP_HUE] = capture.get(CAP_PROP_HUE);

    myVideoCaptureProperties[CAP_PROP_EXPOSURE] = capture.get(CAP_PROP_EXPOSURE);   //曝光
    cout << "CAP_PROP_EXPOSURE:" << myVideoCaptureProperties[CAP_PROP_EXPOSURE] << endl;

    myVideoCaptureProperties[CAP_PROP_FRAME_COUNT] = capture.get(CAP_PROP_FRAME_COUNT);//视频帧数

    myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] = capture.get(CAP_PROP_CONVERT_RGB);//
    cout << "CAP_PROP_CONVERT_RGB:" << myVideoCaptureProperties[CAP_PROP_CONVERT_RGB] << endl;

    //for (int i = 0; i < 40; i++)
    //{
    //	myVideoCaptureProperties[i] = capture.get(i);

    //}

}








