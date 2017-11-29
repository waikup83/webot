/*
 * File:          arm.c
 * Date:          24th May 2011
 * Description:   Implement the functions defined in arm.h
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

#include "arm.h"

#include <webots/robot.h>
#include <webots/motor.h>

#include <stdio.h>
#include <math.h>
#include "reelYB_Struct.h"
#include <sys/time.h> 

static WbDeviceTag arm_elements[5];

static enum Height current_height = ARM_RESET;
static enum Orientation current_orientation = ARM_FRONT;

void arm_init() {
  arm_elements[ARM1] = wb_robot_get_device("arm1");
  arm_elements[ARM2] = wb_robot_get_device("arm2");
  arm_elements[ARM3] = wb_robot_get_device("arm3");
  arm_elements[ARM4] = wb_robot_get_device("arm4");
  arm_elements[ARM5] = wb_robot_get_device("arm5");
  
  wb_motor_set_velocity(arm_elements[ARM2], 0.5);
  
  arm_set_height(ARM_RESET);
  arm_set_orientation(ARM_FRONT);
}

void arm_reset() {
  wb_motor_set_position(arm_elements[ARM1], 0.0);
  wb_motor_set_position(arm_elements[ARM2], 1.57);
  wb_motor_set_position(arm_elements[ARM3], -2.64);
  wb_motor_set_position(arm_elements[ARM4], 1.78);
  wb_motor_set_position(arm_elements[ARM5], 0.0);
}

void arm_set_height(enum Height height) {
  switch (height) {
    case ARM_FRONT_FLOOR:
      wb_motor_set_position(arm_elements[ARM2], -0.97);
      wb_motor_set_position(arm_elements[ARM3], -1.55);
      wb_motor_set_position(arm_elements[ARM4], -0.61);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
       printf("ARM_FRONT_FLOOR\n");
      break;
    case ARM_FRONT_PLATE:
      wb_motor_set_position(arm_elements[ARM2], -0.62);
      wb_motor_set_position(arm_elements[ARM3], -0.98);
      wb_motor_set_position(arm_elements[ARM4], -1.53);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
      printf("ARM_FRONT_PLATE\n");
      break;
    case ARM_FRONT_CARDBOARD_BOX:
      wb_motor_set_position(arm_elements[ARM2],  0.0);
      wb_motor_set_position(arm_elements[ARM3], -0.77);
      wb_motor_set_position(arm_elements[ARM4], -1.21);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
       printf("ARM_FRONT_CARDBOARD_BOX\n");
      break;
    case ARM_RESET:
      wb_motor_set_position(arm_elements[ARM2],  1.57);
      wb_motor_set_position(arm_elements[ARM3], -2.64);
      wb_motor_set_position(arm_elements[ARM4],  1.78);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
      printf("ARM_RESET\n");
      break;
    case ARM_BACK_PLATE_HIGH:
      wb_motor_set_position(arm_elements[ARM2],  0.678);
      wb_motor_set_position(arm_elements[ARM3],  0.682);
      wb_motor_set_position(arm_elements[ARM4],  1.74);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
      printf("ARM_BACK_PLATE_HIGH\n");
      break;
    case ARM_BACK_PLATE_LOW:
      wb_motor_set_position(arm_elements[ARM2], 0.92);
      wb_motor_set_position(arm_elements[ARM3], 0.42);
      wb_motor_set_position(arm_elements[ARM4], 1.78);
      wb_motor_set_position(arm_elements[ARM5],  0.0);
      printf("ARM_BACK_PLATE_LOW\n");
      break;
    case ARM_HANOI_PREPARE:
      wb_motor_set_position(arm_elements[ARM2], -0.4);
      wb_motor_set_position(arm_elements[ARM3], -1.2);
      wb_motor_set_position(arm_elements[ARM4], -M_PI_2);
      wb_motor_set_position(arm_elements[ARM5],  M_PI_2);
      printf("ARM_HANOI_PREPARE\n");
      break;
    default:
      fprintf(stderr, "arm_height() called with a wrong argument\n");
      return;
  }
  current_height = height;
}

void arm_set_orientation(enum Orientation orientation) {
  switch (orientation) {
    case ARM_BACK_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -2.95);
      break;
    case ARM_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -M_PI_2);
      break;
    case ARM_FRONT_LEFT:
      wb_motor_set_position(arm_elements[ARM1], -0.2);
      break;
    case ARM_FRONT:
      wb_motor_set_position(arm_elements[ARM1], 0.0);
      break;
    case ARM_FRONT_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 0.2);
      break;
    case ARM_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], M_PI_2);
      break;
    case ARM_BACK_RIGHT:
      wb_motor_set_position(arm_elements[ARM1], 2.95);
      break;
    default:
      fprintf(stderr, "arm_set_side() called with a wrong argument\n");
      return;
  }
  current_orientation = orientation;
}

void arm_increase_height() {
  current_height++;
  if (current_height >= ARM_MAX_HEIGHT)
    current_height = ARM_MAX_HEIGHT - 1;
  arm_set_height(current_height);
}

void arm_decrease_height() {
  current_height--;
  if ((int) current_height < 0)
    current_height = 0;
  arm_set_height(current_height);
}

void arm_increase_orientation() {
  current_orientation++;
  if (current_orientation >= ARM_MAX_SIDE)
    current_orientation = ARM_MAX_SIDE - 1;
  arm_set_orientation(current_orientation);
}

void arm_decrease_orientation() {
  current_orientation--;
  if ((int) current_orientation < 0)
    current_orientation = 0;
  arm_set_orientation(current_orientation);
}

void arm_set_sub_arm_rotation( enum Arm arm, double radian) {
  wb_motor_set_position(arm_elements[arm], radian);
}

double arm_get_sub_arm_length(enum Arm arm) {
  switch (arm) {
    case ARM1: return 0.253;
    case ARM2: return 0.155;
    case ARM3: return 0.135;
    case ARM4: return 0.081;
    case ARM5: return 0.105;
  }
  return 0.0;
}

void arm_ik(double x, double y, double z) {
  double x1 = sqrt(x*x + z*z);

  double y1 = y + arm_get_sub_arm_length(ARM4) + arm_get_sub_arm_length(ARM5) - arm_get_sub_arm_length(ARM1);
  
  double a = arm_get_sub_arm_length(ARM2);
  double b = arm_get_sub_arm_length(ARM3);
  double c = sqrt(x1*x1 + y1*y1);

  double alpha = - asin( z / x1 );
  double beta = -(M_PI_2 - acos( (a*a + c*c - b*b) / (2.0*a*c) ) - atan( y1/x1 ));
  double gamma = -(M_PI - acos( (a*a + b*b - c*c) / (2.0*a*b) ));
  double delta = -(M_PI + (beta + gamma));
  double epsilon = M_PI_2 + alpha;
  
  //printf(" \n ARM1: %f \n ARM2: %f \n ARM3: %f \n ARM4: %f \n ARM5: %f \n\n", alpha,beta,gamma,delta, epsilon );
  
  printf("X: %f\n Y: %f\n Z: %f\n\n",x,y,z);
  wb_motor_set_position(arm_elements[ARM1], alpha);
  wb_motor_set_position(arm_elements[ARM2], beta);
  wb_motor_set_position(arm_elements[ARM3], gamma);
  wb_motor_set_position(arm_elements[ARM4], delta);
  wb_motor_set_position(arm_elements[ARM5], epsilon);
}

int ARM_TIME_STEP =10;
double ArmGetPosition(unsigned int ArmSection)
{
if(ArmSection <= 4 || ArmSection >=0)
  {
    double Position=0;
    wb_motor_enable_position(arm_elements[ArmSection], ARM_TIME_STEP);
    Position=wb_motor_get_position(arm_elements[ArmSection]);
    return (Position);
  }
 else 
   return (-1.0);
}
void ArmSetPosition(double Position[5])
{
  wb_motor_set_position(arm_elements[ARM1], Position[0]);
  wb_motor_set_position(arm_elements[ARM2], Position[1]);
  wb_motor_set_position(arm_elements[ARM3], Position[2]);
  wb_motor_set_position(arm_elements[ARM4], Position[3]);
  wb_motor_set_position(arm_elements[ARM5], Position[4]);
}


////
////
//// Functions that are compatible with the reel Youbot
////
////


///////////////////////////////////////////////////////////////////////////
//
// Initialize Youbot arm
//
///////////////////////////////////////////////////////////////////////////
YouBotManipulator *reelYB_ArmInit()
{
  arm_init();
  YouBotManipulator *myYouBotManipulator = malloc(sizeof(YouBotManipulator));
  return myYouBotManipulator;
}
///////////////////////////////////////////////////////////////////////////
//
// Exit the arm safely. Dummy for the moment
//
///////////////////////////////////////////////////////////////////////////
void reelYB_ExitArm(YouBotManipulator *myYouBotManipulator)
 {
   wb_robot_cleanup();
  free(myYouBotManipulator);
 }

///////////////////////////////////////////////////////////////////////////
//
// Wait for the arm to reach a particular position
//
///////////////////////////////////////////////////////////////////////////
void reelYB_ArmWaitForPositionReach(YouBotManipulator *myYouBotManipulator,double Position[5], int delay)
{
  const double DELTA = 0.001;  // max tolerated difference
  int i;
  if(delay==-1)
   delay=99999999;
  for(i = 0;i<5;i++){
      wb_motor_enable_position(arm_elements[i], TIME_STEP);
      double effective;  // effective position     
      do 
      {
        wb_robot_step(TIME_STEP);
        delay -= TIME_STEP;
        effective = wb_motor_get_position(arm_elements[i]);
      }
      while (fabs(Position[i] - effective) > DELTA && delay > 0); 
      }
}

///////////////////////////////////////////////////////////////////////////
//
// Move the arm in a particular position from set point
//
///////////////////////////////////////////////////////////////////////////
bool reelYB_ArmSetPosition(YouBotManipulator *myYouBotManipulator,double Position[5], float Speed, bool Wait,int delay)
{
  wb_motor_set_position(arm_elements[ARM1], Position[0]);
  wb_motor_set_position(arm_elements[ARM2], Position[1]);
  wb_motor_set_position(arm_elements[ARM3], Position[2]);
  wb_motor_set_position(arm_elements[ARM4], Position[3]);
  wb_motor_set_position(arm_elements[ARM5], Position[4]);
  
  if(Wait)
   reelYB_ArmWaitForPositionReach(myYouBotManipulator, Position, delay);
    
  
  return true;
}



void reelYB_MoveArmAndBaseByKeyboard(YouBotBase *myYoubotBase)
{

double x=0.2,y=0.1,z=0;
double KinematicStep =0.01;
double ArmPosition[5];
double BasePosition[4];
double ArmPositionStep=0.01;
double BasePositionStep=1;
double BasePositionSpeed=2;
double BaseAngularStep = 5;
double BaseAngularSpeed=2;
struct timeval start, end;
long mtime, seconds, useconds; 
char LastInput='1';
int i,pc = 0;
  wb_robot_keyboard_enable(TIME_STEP);
printf("Begin: MoveArmAndBaseByKeyboard \n\n");
  while (true) 
  {
    step();

    int c = wb_robot_keyboard_get_key();
	
    if (c && c != pc) 
    {
	  //printf("Keyboard press. \n");
      switch (c)
       {
       case 'Y':
           y+= KinematicStep;
           arm_ik( x, y, z);
		   LastInput='y';
           break;
       
        case 'Y'| WB_ROBOT_KEYBOARD_SHIFT:
            y-= KinematicStep;
           arm_ik( x, y, z);
		   LastInput='Y';
           break; 
        case 'X' :
            x+= KinematicStep;
           arm_ik( x, y, z);
		   LastInput='x';
           break; 
        case 'X' | WB_ROBOT_KEYBOARD_SHIFT:
            x-= KinematicStep;
           arm_ik( x, y, z);
		   LastInput='X';
           break;
        case 'Z' :
            z+= KinematicStep;
           arm_ik( x, y, z);
		   LastInput='z';
           break; 
        case 'Z' | WB_ROBOT_KEYBOARD_SHIFT:
            z-= KinematicStep;
           arm_ik( x, y, z);;
		   LastInput='Z';
           break;
        case 'P':
            for(i=0;i<5;i++)
              ArmPosition[i]=ArmGetPosition(i);  
            BaseGetPosition(myYoubotBase,BasePosition)   ;                                 
            printf("\n----ARM: {%f,%f,%f,%f,%f} \n",  ArmPosition[0],ArmPosition[1],ArmPosition[2],ArmPosition[3],ArmPosition[4]);
            printf("\n---BASE: {%f,%f,%f,%f} \n",  BasePosition[0],BasePosition[1],BasePosition[2],BasePosition[3]);
            break;
        case '1':
          ArmPosition[0]=ArmGetPosition(0);
          ArmPosition[0]+=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM1, ArmPosition[0]);
		  LastInput='1';
          break;   
        case '2':
          ArmPosition[0]=ArmGetPosition(0);
          ArmPosition[0]-=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM1, ArmPosition[0]);
		  LastInput='2';
          break; 
         case '3':
          ArmPosition[1]=ArmGetPosition(1);
          ArmPosition[1]+=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM2, ArmPosition[1]);
		  LastInput='3';
          break;   
        case '4':
          ArmPosition[1]=ArmGetPosition(1);
          ArmPosition[1]-=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM2, ArmPosition[1]);
		  LastInput='4';
          break;  
        case '5':
          ArmPosition[2]=ArmGetPosition(2);
          ArmPosition[2]+=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM3, ArmPosition[2]);
		  LastInput='5';
          break;    
        case '6':
          ArmPosition[2]=ArmGetPosition(2);
          ArmPosition[2]-=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM3, ArmPosition[2]);
		  LastInput='6';
          break;    
        case '7':
          ArmPosition[3]=ArmGetPosition(3);
          ArmPosition[3]+=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM4, ArmPosition[3]);
		  LastInput='7';
          break;    
        case '8':
          ArmPosition[3]=ArmGetPosition(3);
          ArmPosition[3]-=ArmPositionStep;
          arm_set_sub_arm_rotation(ARM4, ArmPosition[3]);
		  LastInput='8';
          break; 
        case '9':
          ArmPosition[4]=ArmGetPosition(4);
          ArmPosition[4]+=M_PI_2*ArmPositionStep;
          arm_set_sub_arm_rotation(ARM5, ArmPosition[4]);
		  LastInput='9';
          break;    
        case '0':
          ArmPosition[4]=ArmGetPosition(4);
          ArmPosition[4]-=M_PI_2*ArmPositionStep;
          arm_set_sub_arm_rotation(ARM5, ArmPosition[4]);
		  LastInput='0';
          break;    
        case '+':
		  if(LastInput>='0' && LastInput<='9')
		  {
			ArmPositionStep+=0.01;
			printf("ArmPositionStep: %f\n",ArmPositionStep);
		  }
		  if(LastInput=='F' || LastInput=='B' || LastInput=='L' || LastInput=='R')
		  {
		    BasePositionStep+=1;
			printf("BasePositionStep: %f\n",BasePositionStep);
		  }
		  if(LastInput=='U' || LastInput=='D')
		  {
		    BaseAngularStep +=2;
			printf("BaseAngularStep: %f\n",BaseAngularStep);
          }
          break;  
        case '-':
		 if(LastInput>='0' && LastInput<='9')
		  {
			ArmPositionStep-=0.01;
			printf("ArmPositionStep: %f\n",ArmPositionStep);
		  }
		  if(LastInput=='F' || LastInput=='B' || LastInput=='L' || LastInput=='R')
		  {
		    BasePositionStep-=1;
			printf("BasePositionStep: %f\n",BasePositionStep);
		  }
		  if(LastInput=='U' || LastInput=='D')
		  {
		    BaseAngularStep -=2;
			printf("BaseAngularStep: %f\n",BaseAngularStep);
          }      
          break;  
        case 'T':  
          gettimeofday(&end, NULL);
          seconds  = end.tv_sec  - start.tv_sec;
          useconds = end.tv_usec - start.tv_usec;
          mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
          printf("BaseTimer:: %ld milliseconds\n", mtime);       
          break;                                                                          
        case WB_ROBOT_KEYBOARD_UP:
         // gettimeofday(&start, NULL);
          printf("Go forwards\n");
          reelYB_MoveBaseLongitudinal(myYoubotBase,BasePositionStep, BasePositionSpeed, true,-1);
		  LastInput='F';
          //base_forwards();
          break;
        case WB_ROBOT_KEYBOARD_DOWN:
          //gettimeofday(&start, NULL);
          printf("Go backwards\n");
		  reelYB_MoveBaseLongitudinal(myYoubotBase, -BasePositionStep,BasePositionSpeed, true,-1);
		  LastInput='B';
          //base_backwards();
          break;
        case WB_ROBOT_KEYBOARD_LEFT:
          //gettimeofday(&start, NULL);
          printf("Strafe left\n");
		  reelYB_MoveBaseTransversal(myYoubotBase, -BasePositionStep,BasePositionSpeed, true,-1);
		  LastInput='L';
          //base_strafe_left();
          break;
        case WB_ROBOT_KEYBOARD_RIGHT:
         // gettimeofday(&start, NULL);
          printf("Strafe right\n");
		  reelYB_MoveBaseTransversal(myYoubotBase, BasePositionStep,BasePositionSpeed, true,-1);
		  LastInput='R';
          //base_strafe_right();
          break;
        case WB_ROBOT_KEYBOARD_PAGEUP:
          gettimeofday(&start, NULL);
          printf("Turn left\n");
		  reelYB_MoveBaseAngular(myYoubotBase,  BaseAngularStep, BaseAngularSpeed, true,-1);
		  LastInput='U';
          //base_turn_left();
          break;
        case WB_ROBOT_KEYBOARD_PAGEDOWN:
          gettimeofday(&start, NULL);
          printf("Turn right\n");
		  reelYB_MoveBaseAngular(myYoubotBase,  -BaseAngularStep, BaseAngularSpeed, true,-1);
		  LastInput='D';
          //base_turn_right();
          break;
        case WB_ROBOT_KEYBOARD_END:
        case ' ':
          gettimeofday(&end, NULL);
          seconds  = end.tv_sec  - start.tv_sec;
          useconds = end.tv_usec - start.tv_usec;
          mtime = ((seconds) * 1000 + useconds/1000.0) + 0.5;
          printf("BaseTimer:: %ld milliseconds\n", mtime);       

          printf("Reset\n");
          base_reset();
          arm_reset();
          break;
        case 'G':
        case 388:
        //case 65585:
          printf("Grip\n");
          gripper_grip();
          break;
        case 'G'| WB_ROBOT_KEYBOARD_SHIFT:
        case 390:
          printf("Ungrip\n");
          gripper_release();
          break;
        case 332:
        case WB_ROBOT_KEYBOARD_UP | WB_ROBOT_KEYBOARD_SHIFT:
          printf("Increase arm height\n");
          arm_increase_height();
          break;
        case 326:
        case WB_ROBOT_KEYBOARD_DOWN | WB_ROBOT_KEYBOARD_SHIFT:
          printf("Decrease arm height\n");
          arm_decrease_height();
          break;
        case 330:
        case WB_ROBOT_KEYBOARD_RIGHT | WB_ROBOT_KEYBOARD_SHIFT:
          printf("Increase arm oriwntation\n");
          arm_increase_orientation();
          break;
        case 328:
        case WB_ROBOT_KEYBOARD_LEFT | WB_ROBOT_KEYBOARD_SHIFT:
          printf("Decrease arm orientation\n");
          arm_decrease_orientation();
          break;
        default:
          fprintf(stderr, "Wrong keyboard input(%u)\n",c);
          break;     
      }
    }
    pc = c;
  }
  
printf("End: MoveArmAndBaseByKeyboard \n\n");

}

