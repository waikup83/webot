/*
 * File:          arm.h
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

#ifndef LASER_H
#define LASER_H

#define LASERSIZE 681
#define LASERMINANGLE -2.086214
#define LASERMAXANGLE 2.092350

#define HOKUYO_CALIB 0.0313574
#define CAMERA_TIME_STEP 64

bool OpenLaser();
bool GetLaserData(float *pDAta, float MinAngle, float MaxAngle);
bool CloseLaser();


#endif