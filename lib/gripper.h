/*
 * File:          gripper.h
 * Date:          24th May 2011
 * Description:   Allows to handle the gipper
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

#ifndef GRIPPER_H
#define GRIPPER_H
#include "reelYB_Struct.h"

void gripper_init();

void gripper_grip(); // dangerous to grip an object with this function -> creates a lot of internal constraints
void gripper_release();
void gripper_set_gap(double gap);


////
////
//// Fonction compatible avec le Youbot réel
////
////
bool reelYB_GripperInit(YouBotManipulator *myYouBotManipulator);
bool reelYB_GripperOpen(YouBotManipulator *myYouBotManipulator);
bool reelYB_GripperClose(YouBotManipulator *myYouBotManipulator);
#endif
