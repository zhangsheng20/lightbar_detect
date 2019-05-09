#include "lightbar_detect.h"


//识别对象的一些参数
#define lightbar_length_mm 55.0f                //灯条的长度  单位mm
#define lightbar_distance_mini_mm  135.0f             //小装甲板灯条的宽度   单位mm
//#define lightbar_distance_larger_mm               //大装甲板灯条的宽度   单位mm

//识别条件
#define max_detect_distance_mm  3000.0f         //最远识别距离，超过此距离滤掉  单位mm
#define min_detect_distance_mm  500.0f         //最近识别距离，超过此距离滤掉   单位mm
#define max_inclination_degree   25.0f         //灯条对角线倾斜角超过一定度数，滤掉  单位度
#define max_transformation       0.4f        //

//摄像头的一些参数
#define Camera_fx 6.530675507960873e+02
#define Camera_fy 6.521106670863784e+02
#define Camera_fxy 6.525106670863784e+02


#define Camera_vertical_halfangle  20.0   //1/2垂直方向视角 单位度
#define Camera_lateral_halfangle  20.0   //1/2水平方向视角 单位度
#define Camera_image_area_height_um 2453   //
#define Camera_image_area_width_um 3896




Point2f DetectedArmourYawPitchErrorCurrent[20]={};    //存储所有检测出来的装甲板角度偏差值
Point2f DetectedArmourYawPitchErrorLast[20]={};
Point2f SendYawPitchErrorCurrent=Point2f(0,0);            //存储需要发送的装甲板信息
Point2f SendYawPitchErrorLast=Point2f(0,0);

int  IsDetectedFlag=0 ;                 //是否检测到装甲板信息
extern double myVideoCaptureProperties[50];   //存储摄像头参数



//Distance: the distance of camera and object
//简单的长尺度转换，未考虑相机的畸变和旋转
double CvtRealLenghth2PixelLenghth(double RealLenghth_mm, double Distance_mm)
{
    double PixelLenghth_mm = 0;
    PixelLenghth_mm = Camera_fxy / Distance_mm * RealLenghth_mm;
    return PixelLenghth_mm;
}

//Distance: the distance of camera and object
double CvtPixelLenghth2RealLenghth(double PixelLenghth, double Distance_mm)
{
    double RealLenghth = 0;
    RealLenghth = Distance_mm * PixelLenghth / Camera_fxy;
    return RealLenghth;
}



//计算云台需要转的Yaw和Pitch使得摄像头的中轴线到指定点
Point2f CaculatePitchYawError(float Pixel_x, float Pixel_y)
{
    float PitchAngle = 0;
    float YawAngle = 0;
    float tan_pitch = (Pixel_y - myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] / 2) / Camera_fy;
    float tan_yaw = (Pixel_x - myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] / 2) / Camera_fx;

    PitchAngle = atan(tan_pitch);
    YawAngle = atan(tan_yaw);

    PitchAngle = -PitchAngle / 3.14 * 180; //转化成单位度
    YawAngle = -YawAngle / 3.14 * 180; //转化成单位度
    return Point2f(YawAngle, PitchAngle);
}

Point2f PitchYawError2PixelError(float YawAngleDegree, float PitchAngleDegree)
{
    float YawAngleRad=-YawAngleDegree/180*3.14;  //转化成弧度
    float PitchAngleRad=-PitchAngleDegree/180*3.14;   //转化成弧度

    float tan_yaw=tan(YawAngleRad);
    float tan_pitch=tan(PitchAngleRad);

    float Pixel_x=tan_yaw*Camera_fx+ myVideoCaptureProperties[CAP_PROP_FRAME_WIDTH] / 2 ;
    float Pixel_y=tan_pitch*Camera_fy+myVideoCaptureProperties[CAP_PROP_FRAME_HEIGHT] / 2 ;



return Point2f(Pixel_x,Pixel_y);
}


void DrawEnclosingRexts(Mat &grayImage, Mat &dstImage)
{
    static int IsFirstRun=0;
    if(IsFirstRun==0)
    {
        IsFirstRun=1;
        for(int i=0;i<20;i++)
           {
            DetectedArmourYawPitchErrorCurrent[i]=Point2f(0,0);
           }
    }


    int detected_armour_cnt = 0;

    SendYawPitchErrorLast=SendYawPitchErrorCurrent;
    for(int i=0;i<20;i++) //
    {
        DetectedArmourYawPitchErrorLast[i].x=DetectedArmourYawPitchErrorCurrent[i].x;
        DetectedArmourYawPitchErrorLast[i].y=DetectedArmourYawPitchErrorCurrent[i].y;
    }

    //找出轮廓及最小外接矩形
        vector<vector<Point>> contours;   //每一组Point点集就是一个轮廓
        vector<Vec4i> hierarcy;           //矩形集合

        findContours(grayImage, contours, hierarcy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        vector<RotatedRect> box(contours.size()); //定义最小外接矩形集合

        for (int i = 0; i < contours.size(); i++)
        {
            if(contours[i].size()>30 || contours[i].size()<100)
            {
                box[i] = minAreaRect(Mat(contours[i]));  //计算每个轮廓最小外接矩形
            }
        }
        if (box.empty())
        {
            return;
        }


    //筛选出符合条件的灯条
    //条件一：灯条的长度
    {
        float min_lightbar_PixelLength = CvtRealLenghth2PixelLenghth(lightbar_length_mm, max_detect_distance_mm); //最大距离处灯条的像素长度
        float max_lightbar_PixelLength = CvtRealLenghth2PixelLenghth(lightbar_length_mm, min_detect_distance_mm);

        for (int i = 0; i < box.size(); i++)
        {
            if (box[i].size.height > box[i].size.width)
            {
                if (box[i].size.height < (max_lightbar_PixelLength *(1+max_transformation)) && box[i].size.height >(min_lightbar_PixelLength *(1-max_transformation)))
                {
                    //box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                    //for (int j = 0; j < 4; j++) {

                    //	line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                    //}
                }
            }
            else
            {
                if (box[i].size.width < (max_lightbar_PixelLength *(1+max_transformation)) && box[i].size.width >(1-max_transformation))
                {
                    //box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                    //for (int j = 0; j < 4; j++) {
                    //	line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                    //}
                }
                else
                {
                    box[i].center.x = -1;
                }
            }
        }

    }

    //条件二:灯条的倾斜角度
    for (int i = 0; i < box.size(); i++)
    {
        if (box[i].center.x != -1)
        {
            Point2f rect[4];
           //oint2f   rhombus[4];  //rhombus:内接菱形角点坐标
            box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组


            float err_vertical = abs(rect[0].y - rect[2].y);
            float err_lateral = abs(rect[0].x - rect[2].x);
            float tan_inclination = err_lateral / err_vertical;
            float tan_rect_inclination = tan(max_inclination_degree / 180.0*3.14);
            if (tan_inclination > tan_rect_inclination) {
                box[i].center.x = -1;
            }
            else
            {
                box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                for (int j = 0; j < 4; j++) {
                    line(dstImage, rect[j], rect[(j + 1) % 4], Scalar(0, 0, 255), 1, 8);  //绘制最小外接矩形每条边
                }

            }
        }

    }



    //配对：依据灯条长度和灯条之间距离的比值
    for (int i = 0; i < box.size() - 1; i++)    //最后一个不需要找
    {
        if (box[i].center.x != -1) //是否之前已经滤掉
        {
            //
            float lightbar_PixelLength; //第一个灯条的长度
            if (box[i].size.width > box[i].size.height)
                lightbar_PixelLength = box[i].size.width;
            else
                lightbar_PixelLength = box[i].size.height;

            float real_ratio = lightbar_length_mm / lightbar_distance_mini_mm; //灯条长度和灯条之间距离的比值
            for (int j = i + 1; j < box.size(); j++)  //向后搜索
            {
                if (box[j].center.x != -1)  //是否之前已经滤掉
                {
                    float pixel_ratio = lightbar_PixelLength / (float)(abs(box[i].center.x - box[j].center.x)); //灯条长度和灯条之间像素距离的比值

                    if (pixel_ratio< real_ratio*(max_transformation + 1) && pixel_ratio > real_ratio*(1 - max_transformation))  //判断比值
                    {


                        float lightbar_PixelLength2; //第二个灯条的长度
                        if (box[j].size.width > box[j].size.height)
                            lightbar_PixelLength2 = box[j].size.width;
                        else
                            lightbar_PixelLength2 = box[j].size.height;

                        if (lightbar_PixelLength2 / lightbar_PixelLength > (1 - max_transformation) && lightbar_PixelLength2 / lightbar_PixelLength < (1 + max_transformation)) //两个灯条的长度不能差太多
                        {
                            float err_vertical = abs(box[i].center.y - box[j].center.y);
                            float err_lateral = abs(box[i].center.x - box[j].center.x);
                            float tan_inclination = (err_vertical / err_lateral);   //中心点连线倾斜角

                            Point2f rect[4];
                            Point2f   rhombus[4];  //rhombus:内接菱形角点坐标
                            box[i].points(rect);  //把最小外接矩形四个端点复制给rect数组
                            rhombus[0].x=(rect[0].x+rect[1].x)/2;
                            rhombus[1].x=(rect[1].x+rect[2].x)/2;
                            rhombus[2].x=(rect[2].x+rect[3].x)/2;
                            rhombus[3].x=(rect[3].x+rect[0].x)/2;
                            rhombus[0].y=(rect[0].y+rect[1].y)/2;
                            rhombus[1].y=(rect[1].y+rect[2].y)/2;
                            rhombus[2].y=(rect[2].y+rect[3].y)/2;
                            rhombus[3].y=(rect[3].y+rect[0].y)/2;

                             err_vertical = abs(rect[0].y - rect[2].y);
                             err_lateral = abs(rect[0].x - rect[2].x);
                            float tan_inclination2= err_lateral / err_vertical; //灯条倾斜角

                            if (tan_inclination < tan(max_inclination_degree / 180.0*3.14)) //判断灯条中心连线的倾斜角
                            {
                                //if(tan_inclination*tan_inclination2>(1-max_transformation)&&tan_inclination*tan_inclination2<(1+max_transformation)) //判断两个平行的灯条是否错位

                                {
                                    float dis_x = (int)((box[i].center.x + box[j].center.x) / 2);
                                    float dis_y = (int)((box[i].center.y + box[j].center.y) / 2);
                                    circle(dstImage, Point(dis_x, dis_y), 5, (0, 0, 255), 1);
                                    Point2f PitchYawError = CaculatePitchYawError(dis_x, dis_y);
                                    //cout << PitchYawError.x << "   " << PitchYawError.y << endl;

                                    DetectedArmourYawPitchErrorCurrent[detected_armour_cnt] = PitchYawError;
                                    detected_armour_cnt++;
                                }

                            }
                        }



                    }
                }
            }
        }

    }



    if (detected_armour_cnt == 0)
    {
        IsDetectedFlag=0;   //未检测到
       // SendYawPitchError = Point2f(0,0);
    }
    else
    { //找出离上一帧最近的点
         IsDetectedFlag=1;   //检测到
         float min_error = 200;
        for (int i = 0; i < detected_armour_cnt;i++)
        {
            float error = pow((DetectedArmourYawPitchErrorCurrent[i].x-SendYawPitchErrorLast.x),2)
                            +pow((DetectedArmourYawPitchErrorCurrent[i].y-SendYawPitchErrorLast.y),2);
             if (error < min_error)
             {

                  min_error = error;
                  SendYawPitchErrorCurrent =DetectedArmourYawPitchErrorCurrent[i];               
             }

        }
        Point2f Pixel= PitchYawError2PixelError(SendYawPitchErrorCurrent.x,SendYawPitchErrorCurrent.y);
        circle(dstImage, Point(Pixel.x, Pixel.y), 10, (0, 0, 255), 4);

    }





//    {//找出离中心最近的装甲板，发送给下位机
//        IsDetectedFlag=1;   //检测到
//      float max_error = 0;
//        for (int i = 0; i < detected_armour_cnt;i++)
//       {
//           float error = DetectedArmourYawPitchError[i].x*DetectedArmourYawPitchError[i].x +
//              DetectedArmourYawPitchError[i].y*DetectedArmourYawPitchError[i].y*0.5;   //减小垂直方向的权重
//         if (error > max_error)
//        {
//           max_error = error;
//         SendYawPitchError =DetectedArmourYawPitchError[i];
//   }

//    }
//}

    //cout << SendYawPitchErrorCurrent.x << "   " << SendYawPitchErrorCurrent.y << endl;

}

float GetPixelLength(Point PixelPointO, Point PixelPointA)
{
    float PixelLength;
    PixelLength = powf((PixelPointO.x - PixelPointA.x), 2) + powf((PixelPointO.y - PixelPointA.y), 2);
    PixelLength = sqrtf(PixelLength);
    return PixelLength;
}
