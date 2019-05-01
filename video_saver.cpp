#include "video_saver.h"


string GetCurrentTime();
string IntToString(int value);
VideoSaver::VideoSaver(int num)
{
    frameCount = 0;
    maxFrame = num;
}

string IntToString(int value)
{
    ostringstream convert;
    convert << value;
    return convert.str();
}


string GetCurrentTime()
{
    time_t Time = time(NULL);
    tm* LocalTime = localtime(&Time);
    string Result;

    // add the year
    Result += IntToString(LocalTime->tm_year + 1900) + "-";
    // add the month
    Result += IntToString(LocalTime->tm_mon + 1) + "-";
    // add the day
    Result += IntToString(LocalTime->tm_mday) + "_";
    // add the hour
    Result += IntToString(LocalTime->tm_hour) + "-";
    // add the minutes
    Result += IntToString(LocalTime->tm_min) + "-";
    // add the seconds
    Result += IntToString(LocalTime->tm_sec);

    return Result;
}


void VideoSaver::write(Mat frame)
{
    if( frameCount % maxFrame == 0 )
    {
        string fileName = "/home/sheng/桌面" + GetCurrentTime() + ".avi";
        writer.open(fileName, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, frame.size());
    }
    frameCount++;
    writer.write(frame);
}



string GetCurrentTime2()
{
    time_t Time = time(NULL);
    tm* LocalTime = localtime(&Time);
    string Result;

    // add the year
    Result += IntToString(LocalTime->tm_year + 1900) + "-";
    // add the month
    Result += IntToString(LocalTime->tm_mon + 1) + "-";
    // add the day
    Result += IntToString(LocalTime->tm_mday) + " ";
    // add the hour
    Result += IntToString(LocalTime->tm_hour) + ":";
    // add the minutes
    Result += IntToString(LocalTime->tm_min) + ":";
    // add the seconds
    Result += IntToString(LocalTime->tm_sec);

    return Result;
}
