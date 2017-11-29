/*
 * File:          youbot.c
 * Date:          24th May 2011
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 * Author:        fabien.rohrer@cyberbotics.com
 * Modifications: 
 */
#include "YoubotWebotsAllInclude.h"


float LastHokuyo[LASERSIZE];
float HokuyoData[LASERSIZE];

void GetHokuyoData(float *pData)
{
  WbDeviceTag urg04lx = wb_robot_get_device("URG-04LX-UG01");
  wb_camera_enable(urg04lx,CAMERA_TIME_STEP);
  wb_robot_step(CAMERA_TIME_STEP);
  float *Data= wb_camera_get_range_image(urg04lx);
  memcpy(pData,Data,wb_camera_get_width(urg04lx)*sizeof(float));
  wb_camera_disable(urg04lx);
}
void  PrintHokuyoData()
{
// get the devices
  float Data[LASERSIZE];
  OpenLaser();
  GetLaserData( Data, LASERMINANGLE, LASERMAXANGLE);
  
  printf("\n\n");
  printf("Hukuyo width: %i \n",LASERSIZE);

  printf(" Center data: %f\n",*(Data+(LASERSIZE/2)));
  printf(" Center diff: %f\n\n",LastHokuyo[LASERSIZE/2]-*((Data+(LASERSIZE/2))));

  int i=0;
  for(i=0;i<LASERSIZE;i++)
  {
   printf (" %f ", *(Data+i));
   LastHokuyo[i]=*(Data+i);
   if(i%23==0 && i!=0)
     printf("\n");
  }

 CloseLaser();
}

int main(int argc, char **argv)
{
 // Arm, base and gripper initialization
 
  YouBotBase *myYouBotBase=0;
  reelYB_Init(myYouBotBase);
  myYouBotBase=reelYB_BaseInit();
  if(myYouBotBase==0)
    return -1;
    
  YouBotManipulator *MyYouBotManipulator=0;
  MyYouBotManipulator=reelYB_ArmInit();
  if(MyYouBotManipulator==0)
      return -1;
  
  reelYB_GripperInit(MyYouBotManipulator);
     
     
  // Move arm and robot       
  passive_wait(2.0);
  float Distance =10.0;
  float Speed =2.0;
 
 base_strafe_right() ;
   passive_wait(5.0);
 //reelYB_MoveBaseTransversal(myYouBotBase,  Distance, Speed, true,-1);
  //reelYB_MoveBaseAngular(myYouBotBase,  -180, Speed, true,-1);
  /*reelYB_ArmSetPosition(MyYouBotManipulator,ArmPositionbloc1[1],Speed,false,-1);
  reelYB_MoveBaseLongitudinal(myYouBotBase,  Distance, Speed, true,-1);
  reelYB_ArmSetPosition(MyYouBotManipulator,ArmPositionbloc1[5],Speed,false,-1);
  reelYB_GripperOpen(MyYouBotManipulator);
  reelYB_GripperClose(MyYouBotManipulator);
  PrintHokuyoData();
  reelYB_MoveArmAndBaseByKeyboard(myYouBotBase);
  // Exit the robot 
*/  

  reelYB_ExitBase(myYouBotBase);
  reelYB_ExitArm(MyYouBotManipulator);
  
  return 0;
}
