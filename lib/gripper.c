/*
 * File:          gripper.c
 * Date:          24th May 2011
 * Description:   Implement the functions defined in gripper.h
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

#include "gripper.h"

#include <webots/robot.h>
#include <webots/motor.h>

#include "tiny_math.h"

#define LEFT 0
#define RIGHT 1

#define MIN_POS 0.0
#define MAX_POS 0.025
#define OFFSET_WHEN_LOCKED 0.021

static WbDeviceTag fingers[2];

void gripper_init() {
  fingers[LEFT] = wb_robot_get_device("finger1");
  fingers[RIGHT] = wb_robot_get_device("finger2");

  wb_motor_set_velocity(fingers[LEFT], 0.03);
  wb_motor_set_velocity(fingers[RIGHT], 0.03);
}

void gripper_grip() {
  wb_motor_set_position(fingers[LEFT], MIN_POS);
  wb_motor_set_position(fingers[RIGHT], MIN_POS);
}

void gripper_release() {
  wb_motor_set_position(fingers[LEFT], MAX_POS);
  wb_motor_set_position(fingers[RIGHT], MAX_POS);
}

void gripper_set_gap(double gap) {
  double v = bound(0.5*(gap - OFFSET_WHEN_LOCKED), MIN_POS, MAX_POS);
  wb_motor_set_position(fingers[LEFT],  v);
  wb_motor_set_position(fingers[RIGHT], v);
}

///////////////////////////////////////////////////////////////////////////
//
// Init the gripper
//
///////////////////////////////////////////////////////////////////////////
bool  reelYB_GripperInit(YouBotManipulator *myYouBotManipulator)
{
  gripper_init();
  return true;
}
///////////////////////////////////////////////////////////////////////////
//
// Open the gripper
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_GripperOpen(YouBotManipulator *myYouBotManipulator)
{
  gripper_release();
  passive_wait(2.0);
  return true;
}
///////////////////////////////////////////////////////////////////////////
//
// Close the gripper
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_GripperClose(YouBotManipulator *myYouBotManipulator)
{
  gripper_grip();
  passive_wait(2.0);
  return true;
}
