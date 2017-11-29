/*
 * File:          base.h
 * Date:          24th May 2011
 * Description:   Allows to handle the base
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

#ifndef BASE_H
#define BASE_H

#include <webots/types.h>
#include "reelYB_Struct.h"
enum Wheels { WHEEL1, WHEEL2, WHEEL3, WHEEL4 };
void base_init();

void base_reset();
void base_forwards();
void base_backwards();
void base_turn_left();
void base_turn_right();
void base_strafe_left();
void base_strafe_right();

void base_goto_init(double time_step);
void base_goto_set_target(double x, double z, double a);
void base_goto_run();
bool base_goto_reached();




#define TIME_STEP 32
bool   reelYB_Init();
bool   reelYB_ExitBase(YouBotBase *myYouBotBase);
YouBotBase *reelYB_BaseInit();
bool reelYB_MoveBaseLongitudinal(YouBotBase *MyYoubotBase, float Distance, float Speed, bool Wait,int delay);
bool reelYB_MoveBaseAngular(YouBotBase* myYoubotBase, float TheDistance, float Speed, bool Wait,int delay);
bool reelYB_MoveBaseTransversal(YouBotBase* myYoubotBase, float Distance, float Speed, bool Wait,int delay);

void BaseWaitForWheelReach(YouBotBase *myYouBotBase, double Position,int WheelNumber, int delay);
void BaseWaitForPositionReachYouBotBase (YouBotBase *MyYoubotBase,double Position[4], int delay);
bool BaseGetPosition(YouBotBase *MyYoubotBase,double Position[4]);


bool reelYB_StopBase(YouBotBase* myYouBotBase);
bool reelYB_BaseGoForward(YouBotBase* myYouBotBase, float Speed);
bool reelYB_BaseGoBackward(YouBotBase* myYouBotBase, float Speed);
bool reelYB_BaseStrafLeft(YouBotBase* myYouBotBase,  float Speed);
void reelYB_MoveBaseByKeyboard(YouBotBase* *myYouBotBase, float Speed);
bool reelYB_BaseStrafRight(YouBotBase* myYouBotBase, float Speed);
bool reelYB_BaseRotateLeft(YouBotBase* myYouBotBase, float Speed);
bool reelYB_BaseRotateRight(YouBotBase* myYouBotBase, float Speed);


#endif
