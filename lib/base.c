/*
 * File:          base.c
 * Date:          24th May 2011
 * Description:   Implement the functions defined in base.h
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

#include "base.h"

#include "tiny_math.h"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/compass.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define SPEED 4.0
#define DISTANCE_TOLERANCE 0.001
#define ANGLE_TOLERANCE 0.001

// stimulus coefficients
#define K1 3.0
#define K2 1.0
#define K3 1.0

typedef struct {
  Vector2 v_target;
  double alpha;
  bool reached;
} goto_struct;

static WbDeviceTag wheels[4];
static WbDeviceTag gps;
static WbDeviceTag compass;
static goto_struct goto_data;

static void base_set_wheel_velocity(WbDeviceTag t, double velocity) {
  if (velocity < 0.0)
    wb_motor_set_position(t, -INFINITY);
  else
    wb_motor_set_position(t, INFINITY);
  
  wb_motor_set_velocity(t, fabs(velocity));
}

static void base_set_wheel_speeds_helper(double speeds[4]) {
  int i;
  for (i=0; i<4; i++)
    base_set_wheel_velocity(wheels[i], speeds[i]);
}


void base_init() {
  int i;
  char wheel_name[16];
  for (i=0; i<4; i++) {
    sprintf(wheel_name, "wheel%d", (i+1));
    wheels[i] = wb_robot_get_device(wheel_name);
  }
}

void base_reset() {
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);  
}

void base_forwards() {
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_backwards() {
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
}

void base_turn_left() {
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);  
}

void base_turn_right() {
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);  
}

void base_strafe_left() {
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);  
}

void base_strafe_right() {
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);  
}

void base_goto_init(double time_step) {
  gps = wb_robot_get_device("gps");
  compass = wb_robot_get_device("compass");
  if (gps)
    wb_gps_enable(gps, time_step);
  if (compass)
    wb_compass_enable(compass, time_step);
  if (!gps || !compass)
    fprintf(stderr, "cannot use goto feature without GPS and Compass");
  
  goto_data.v_target.u = 0.0;
  goto_data.v_target.v = 0.0;
  goto_data.alpha = 0.0;
  goto_data.reached = false;
}

void base_goto_set_target(double x, double z, double alpha) {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  goto_data.v_target.u = x;
  goto_data.v_target.v = z;
  goto_data.alpha = alpha;
  goto_data.reached = false;
}

void base_goto_run() {
  if (!gps || !compass)
    fprintf(stderr, "base_goto_set_target: cannot use goto feature without GPS and Compass");

  // get sensors
  const double *gps_raw_values = wb_gps_get_values(gps);
  const double *compass_raw_values = wb_compass_get_values(compass);
  
  // compute 2d vectors
  Vector2 v_gps = { gps_raw_values[0], gps_raw_values[2] };
  Vector2 v_front = { compass_raw_values[0], compass_raw_values[1] };
  Vector2 v_right = { -v_front.v, v_front.u };
  Vector2 v_north = { 1.0, 0.0 };
  
  // compute distance
  Vector2 v_dir;
  vector2_minus(&v_dir, &goto_data.v_target, &v_gps);
  double distance = vector2_norm(&v_dir);
  
  // compute absolute angle & delta with the delta with the target angle
  double theta = vector2_angle( &v_front, &v_north );
  double delta_angle = theta - goto_data.alpha;
  
  // compute the direction vector relatively to the robot coordinates
  // using an a matrix of homogenous coordinates
  Matrix33 transform;
  matrix33_set_identity(&transform);
  transform.a.u = v_front.u;
  transform.a.v = v_right.u;
  transform.b.u = v_front.v;
  transform.b.v = v_right.v;
  transform.c.u = -v_front.u*v_gps.u -v_front.v*v_gps.v;
  transform.c.v = -v_right.u*v_gps.u -v_right.v*v_gps.v;
  Vector3 v_target_tmp = {goto_data.v_target.u, goto_data.v_target.v, 1.0};
  Vector3 v_target_rel;
  matrix33_mult_vector3(&v_target_rel, &transform, &v_target_tmp);
  
  // compute the speeds
  double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  // -> first stimulus: delta_angle
  speeds[0] = -delta_angle / M_PI * K1;
  speeds[1] =  delta_angle / M_PI * K1;
  speeds[2] = -delta_angle / M_PI * K1;
  speeds[3] =  delta_angle / M_PI * K1;
  
  // -> second stimulus: u coord of the relative target vector
  speeds[0] +=  v_target_rel.u * K2;
  speeds[1] +=  v_target_rel.u * K2;
  speeds[2] +=  v_target_rel.u * K2;
  speeds[3] +=  v_target_rel.u * K2;

  // -> third stimulus: v coord of the relative target vector
  speeds[0] +=  -v_target_rel.v * K3;
  speeds[1] +=   v_target_rel.v * K3;
  speeds[2] +=   v_target_rel.v * K3;
  speeds[3] +=  -v_target_rel.v * K3;

  // apply the speeds
  int i;
  for (i=0; i<4; i++) {
    speeds[i] /= (K1 + K2 + K2); // number of stimuli (-1 <= speeds <= 1)
    speeds[i] *= SPEED; // map to speed (-SPEED <= speeds <= SPEED)
    
    // added an arbitrary factor increasing the convergence speed
    speeds[i] *= 30.0;
    speeds[i]  = bound (speeds[i], -SPEED, SPEED);
  }
  base_set_wheel_speeds_helper(speeds);
  
  // check if the taget is reached
  if (distance    < DISTANCE_TOLERANCE &&
      delta_angle <  ANGLE_TOLERANCE &&
      delta_angle > -ANGLE_TOLERANCE)
    goto_data.reached = true;
}

bool base_goto_reached() {
  return goto_data.reached;
}


////
////
//// Functions that are compatible with the reel Youbot
////
////

int BASE_TIME_STEP =32;
///////////////////////////////////////////////////////////////////////////
//
// Initialize the robot
//
///////////////////////////////////////////////////////////////////////////
bool  reelYB_Init()
{
  wb_robot_init();
  return true;
}
///////////////////////////////////////////////////////////////////////////
//
// Initialize Youbot base
//
///////////////////////////////////////////////////////////////////////////
YouBotBase *reelYB_BaseInit()
{
  YouBotBase *myYouBotBase = malloc(sizeof(YouBotBase));
  double Position[4];
  base_init();
  BaseGetPosition(myYouBotBase,Position);
  return myYouBotBase;
}

///////////////////////////////////////////////////////////////////////////
//
// Exit the base safely by waiting the thread that handle the Youbot base move
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_ExitBase(YouBotBase *myYouBotBase)
{
  wb_robot_cleanup();
  free(myYouBotBase);
   return true;
}

//
// Internal fonction
//
// Get the position of all whell of the base
//
//
bool BaseGetPosition(YouBotBase *myYoubotBase,double Position[4])
{
  int i;
  for(i = 0;i<4;i++)
  {
      wb_motor_enable_position(wheels[i], TIME_STEP);
      Position[i] = wb_motor_get_position(wheels[i]);
  }
  return true;
}


//
// Internal fonction
//
// Wait for the base to reach a particular position
// This function is not available because there is no conterpart function in the reel Youbot library.
// This function do not exist in the reel Youbot library because it's implementation is made with thread
// in the reel Youbot.
//
void BaseWaitForPositionReach(YouBotBase *myYouBotBase, double Position[4], int delay)
{
  const double DELTA = 0.001;  // max tolerated difference
  int i;
  
  if(delay==-1)
   delay=999999999;
  for(i = 0;i<4;i++){
      wb_motor_enable_position(wheels[i], TIME_STEP);
      double effective;  // effective position
      do {
        wb_robot_step(TIME_STEP);
        delay -= TIME_STEP;
        effective = wb_motor_get_position(wheels[i]);
        }
      while (fabs(Position[i] - effective) > DELTA && delay > 0);
      }
}


///
// Internal fonction
//
// Wait for a wheel to reach a particular position
// This function is not available because there is no conterpart function in the reel Youbot library.
// This function do not exist in the reel Youbot library because it's implementation is made with thread
// in the reel Youbot.
//
void BaseWaitForWheelReach(YouBotBase *myYouBotBase, double Position,int WheelNumber, int delay)
{
  double effective;  // effective position
  const double DELTA = 0.001;  // max tolerated difference


  if(delay==-1)
   delay=99999;
 
    wb_motor_enable_position(wheels[WheelNumber], TIME_STEP);
    wb_robot_step(TIME_STEP);
    
    do 
    {

      
      delay -= TIME_STEP;
      effective = wb_motor_get_position(wheels[WheelNumber]);
      wb_robot_step(TIME_STEP); 
    }
    while (fabs(Position - effective) > DELTA && delay > 0);
      
}

///////////////////////////////////////////////////////////////////////////
//
// Move robot from backward or forward
// 
// N.B. Parameter int delay is not use in the reel Youbot version
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_MoveBaseLongitudinal(YouBotBase *myYoubotBase, float Distance, float Speed, bool Wait,int delay)
{
  
  
  int i;
  Distance=(Distance*2*M_PI)/10;
  for(i = 0;i<4;i++)
  {
      base_set_wheel_velocity(wheels[i], Speed);
      wb_motor_enable_position(wheels[i], TIME_STEP);
      double effective = wb_motor_get_position(wheels[i]);
      wb_motor_set_position(wheels[i],effective + Distance);
  }
  
   if(Wait)
   {
     double Position[4];
     for(i=0;i<4;i++)
        Position[i]=wb_motor_get_position(wheels[i])+ Distance;
     BaseWaitForPositionReach(myYoubotBase, Position, delay);
   }
    return true;
}

///////////////////////////////////////////////////////////////////////////
//
// Rotate robot on itself for a certain amount of degre(not rad).
// 
// N.B. Parameter int delay is not use in the reel Youbot version
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_MoveBaseAngular(YouBotBase* myYoubotBase, float pAngle, float Speed, bool Wait,int delay)
{
 double Position[4];

  float  Angle= pAngle*0.147;//0.202444;//(Distance*2*M_PI)/10; // 1 dergre is about 0.202444cm of wheel displacment
  fprintf(stderr, "Angle: %f \n",  Angle);

  base_set_wheel_velocity(wheels[0], -Speed);
  wb_motor_enable_position(wheels[0], TIME_STEP);
  double effective = wb_motor_get_position(wheels[0]);
  wb_motor_set_position(wheels[0],effective - Angle);
  Position[0]=wb_motor_get_position(wheels[0])-  Angle;

  base_set_wheel_velocity(wheels[1], Speed);
  wb_motor_enable_position(wheels[1], TIME_STEP);
  effective = wb_motor_get_position(wheels[1]);
  wb_motor_set_position(wheels[1],effective +  Angle);
  Position[1]=wb_motor_get_position(wheels[1])+  Angle;
  
  
  base_set_wheel_velocity(wheels[2], -Speed); 
  wb_motor_enable_position(wheels[2], TIME_STEP);
  effective = wb_motor_get_position(wheels[2]);
  wb_motor_set_position(wheels[2],effective -  Angle);
  Position[2]=wb_motor_get_position(wheels[2])-  Angle;
  
  base_set_wheel_velocity(wheels[3], Speed);
  wb_motor_enable_position(wheels[3], TIME_STEP);
  effective = wb_motor_get_position(wheels[3]);
  wb_motor_set_position(wheels[3],effective +  Angle);
  Position[3]=wb_motor_get_position(wheels[3])+  Angle;
  
   if(Wait)
   {
     BaseWaitForPositionReach(myYoubotBase, Position, delay);
   }
    return true;
    
}


///////////////////////////////////////////////////////////////////////////
//
// Stop the base
//
///////////////////////////////////////////////////////////////////////////
bool  reelYB_StopBase(YouBotBase* myYouBotBase)
{
  static double speeds[4] = {0.0, 0.0, 0.0, 0.0};
  base_set_wheel_speeds_helper(speeds);  
   return true;
}
///////////////////////////////////////////////////////////////////////////
//
// Move robot forward
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseGoForward(YouBotBase* myYouBotBase, float Speed)
//void base_forwards() 
{
  static double speeds[4] = {SPEED, SPEED, SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds);
   return true;
}


///////////////////////////////////////////////////////////////////////////
//
// Move robot backward
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseGoBackward(YouBotBase* myYouBotBase, float Speed)
{
  static double speeds[4] = {-SPEED, -SPEED, -SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
  return true;
}


///////////////////////////////////////////////////////////////////////////
//
// Rotate left
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseRotateLeft(YouBotBase* myYouBotBase, float Speed)
{
  static double speeds[4] = {SPEED, -SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);
   return true;  
}

///////////////////////////////////////////////////////////////////////////
//
// Rotate right
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseRotateRight(YouBotBase* myYouBotBase, float Speed)
{
  static double speeds[4] = {-SPEED, SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds); 
   return true; 
}

///////////////////////////////////////////////////////////////////////////
//
// Starf left
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseStrafLeft(YouBotBase* myYouBotBase, float Speed)
{
  static double speeds[4] = {SPEED, -SPEED, -SPEED, SPEED};
  base_set_wheel_speeds_helper(speeds); 
   return true; 
}

///////////////////////////////////////////////////////////////////////////
//
// Straf right
// 
///////////////////////////////////////////////////////////////////////////
bool reelYB_BaseStrafRight(YouBotBase* myYouBotBase, float Speed)
{
  static double speeds[4] = {-SPEED, SPEED, SPEED, -SPEED};
  base_set_wheel_speeds_helper(speeds);  
   return true;
}


///////////////////////////////////////////////////////////////////////////
//
// Move robot from side to side.
// 
// N.B. Parameter int delay is not use in the reel Youbot version
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_MoveBaseTransversal(YouBotBase* myYoubotBase, float Distance, float Speed, bool Wait,int delay)
{


 double Position[4];


  Distance=Distance * 0.21842105263; // 1cm of transversal dispplacment is about 0.21842105263 cm of wheel displacment
  fprintf(stderr, "Distance: %f \n", Distance);
 
  base_set_wheel_velocity(wheels[0], -Speed);
  wb_motor_enable_position(wheels[0], TIME_STEP);
  double effective = wb_motor_get_position(wheels[0]);
  wb_motor_set_position(wheels[0],effective - Distance);
  Position[0]=wb_motor_get_position(wheels[0])- Distance;

  base_set_wheel_velocity(wheels[1], Speed);
  wb_motor_enable_position(wheels[1], TIME_STEP);
  effective = wb_motor_get_position(wheels[1]);
  wb_motor_set_position(wheels[1],effective + Distance);
  Position[1]=wb_motor_get_position(wheels[1])+ Distance;
  
  
  base_set_wheel_velocity(wheels[2], Speed); 
  wb_motor_enable_position(wheels[2], TIME_STEP);
  effective = wb_motor_get_position(wheels[2]);
  wb_motor_set_position(wheels[2],effective + Distance);
  Position[2]=wb_motor_get_position(wheels[2])+ Distance;
  
  base_set_wheel_velocity(wheels[3], -Speed);
  wb_motor_enable_position(wheels[3], TIME_STEP);
  effective = wb_motor_get_position(wheels[3]);
  wb_motor_set_position(wheels[3],effective - Distance);
  Position[3]=wb_motor_get_position(wheels[3])- Distance;
  
   if(Wait)
   {
     BaseWaitForPositionReach(myYoubotBase, Position, delay);
   }
    return true;
    
}




