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

#ifndef ARM_H
#define ARM_H
#include "reelYB_Struct.h"

void arm_init();

void arm_reset();

enum Height {
  ARM_FRONT_FLOOR, ARM_FRONT_PLATE, ARM_HANOI_PREPARE, ARM_FRONT_CARDBOARD_BOX,
  ARM_RESET, ARM_BACK_PLATE_HIGH, ARM_BACK_PLATE_LOW, ARM_MAX_HEIGHT
};
void arm_set_height(enum Height height);
void arm_increase_height();
void arm_decrease_height();

enum Orientation {
  ARM_BACK_LEFT, ARM_LEFT, ARM_FRONT_LEFT, ARM_FRONT, ARM_FRONT_RIGHT,
  ARM_RIGHT, ARM_BACK_RIGHT, ARM_MAX_SIDE
};
void arm_set_orientation(enum Orientation orientation);
void arm_increase_orientation();
void arm_decrease_orientation();

enum Arm { ARM1, ARM2, ARM3, ARM4, ARM5 };
void arm_set_sub_arm_rotation( enum Arm arm, double radian );
double arm_get_sub_arm_length(enum Arm arm);

void arm_ik(double x, double y, double z);
double ArmGetPosition(unsigned int ArmSection);
void ArmSetPosition(double Position[5]);


////
////
//// Fonction compatible avec le Youbot réel
////
////



YouBotManipulator *reelYB_ArmInit();
bool reelYB_ArmSetPosition(YouBotManipulator *myYouBotManipulator,double Position[5], float Speed, bool Wait,int delay);
void reelYB_ArmWaitForPositionReach(YouBotManipulator *myYouBotManipulator,double Position[5], int delay);
void reelYB_MoveArmAndBaseByKeyboard(YouBotBase *myYoubotBase);
void reelYB_ExitArm();
#endif
