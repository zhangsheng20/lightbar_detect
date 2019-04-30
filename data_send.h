#ifndef DATA_SEND_H
#define DATA_SEND_H

#include <iostream>

#include <stdio.h>
#include "stdlib.h"
#include "serial.h"

#include "opencv2/core/core.hpp"

#include "opencv2/imgproc/imgproc.hpp"

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"



using namespace std;
using namespace cv;

void SendDataToInfantry();
void Data_disintegrate_u16(unsigned int Data, unsigned char *LData,
                       unsigned char *HData);
void Data_disintegrate_s16(int Data, unsigned char *LData,
                       unsigned char *HData);

unsigned char Add_CRC(unsigned char InputBytes[], unsigned char data_lenth) ;
void Data_Code( int x_Data,  int y_Data);



#endif
