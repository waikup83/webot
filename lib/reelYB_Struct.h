#ifndef REELYB_STRUCT_H
#define REELYB_STRUCT_H

//// 
//// Home made add-on made by Gilles Champagne
//// Teacher at the Computer science department of Cégep Lévis Lauzon, Lévis, Quebec Canada
////
//// This e file contains an additional function that is compatible with the reel Youbot system.
//// See function at the bottom of that file.
//// 


#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h> 


#define TIME_STEP 32

static void step() 
{
  if (wb_robot_step(TIME_STEP) == -1) 
  {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}
static void passive_wait(double sec) 
{
  double start_time = wb_robot_get_time();
  do 
  {
    step();
  }
  while(start_time + sec > wb_robot_get_time());
}
typedef struct 
{
/*
  WbDeviceTag wheels[4];
  WbDeviceTag gps;
  WbDeviceTag compass;
  bool RobotHasArm;
  bool RobotHasBase;
  bool RobotHasGripper;
  */
} YouBotBase; 

typedef struct 
{
/*
  WbDeviceTag wheels[4];
  WbDeviceTag gps;
  WbDeviceTag compass;
  bool RobotHasArm;
  bool RobotHasBase;
  bool RobotHasGripper;
  */
} YouBotManipulator; 
#endif