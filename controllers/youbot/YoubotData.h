#ifndef YOUBOT_DATA_H
#define  YOUBOT_DATA_H

#ifdef WEBOTS_APPLICATION
double ArmPositionbloc1[3][5]= {{-0.295552,0.919454,0.439895,1.744200,-0.000000} ,
                                {-0.028815,1.119837,0.043186,1.770702,0.054590} ,
                                {0.290871,0.976683,0.314007,1.780240,0.216478} };
// Avec le scan
//double PositionBrasBoite[5] = { -0.028815,-0.919766,0.074561,-0.789131,0.059663 };
// Sans le scan
double PositionBrasBoite[5] = { 0.002493,-0.370556,-0.901162,-0.840926,0.176524 };
#endif

#ifdef REELYOUBOT_APPLICATION   


double ArmPositionReady[5]= {302335,126052,181068,23004,81462} ; 
double ArmPositionAppPick[5]={302335.000000,21877.000000,189136.000000,1522.000000,81462.000000};
double ArmPositionPick[5]={302335,10924,184142 ,1522 ,81462} ;                         
double ArmPositionAppDrop[5]={302335.000000,216485.000000,119579.000000,144529.000000,66146.000000};
double ArmPositionDrop[5]={302335.000000,225987.000000,115578.000000,138531.000000,66146.000000};


#endif                     
                      
#endif