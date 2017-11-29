/*
 * File:          arm.c
 * Date:          24th May 2011
 * Description:   Allows to handle the arm
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */

//// 
//// Home made add-on made by Gilles Champagne
//// Teacher at the Computer science department of Cégep Lévis Lauzon, Lévis, Quebec Canada
////
//// This file contains an additional function that is compatible with the reel Youbot system.
//// See function at the bottom of that file.
//// 

/*#include "laser.h"*/


#define LASERSIZE 681
#define LASERMINANGLE -2.086214
#define LASERMAXANGLE 2.092350

#define HOKUYO_CALIB 0.0313574
#define CAMERA_TIME_STEP 64


WbDeviceTag urg04lx;
bool OpenLaser()
{
  urg04lx = wb_robot_get_device("URG-04LX-UG01");
  wb_camera_enable(urg04lx,CAMERA_TIME_STEP);
  wb_robot_step(CAMERA_TIME_STEP);
  return true;
}

bool GetLaserData(float *pData, float MinAngle, float MaxAngle)
{
  float *Data= wb_camera_get_range_image(urg04lx);
 // memcpy(pData,Data,wb_camera_get_width(urg04lx)*sizeof(float));
  
  int i=0;
  for(i=0;i<LASERSIZE;i++)
  {
   *(pData+i)=*(Data+i);
  }
   return true;
}
bool CloseLaser()
{
  wb_camera_disable(urg04lx);
  return true;
}


