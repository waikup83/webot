#ifndef YOUBOT_DATA_H
#define  YOUBOT_DATA_H

#ifdef WEBOTS_APPLICATION
double ArmPositionbloc1[8][5]= {{-0.000004,-0.153591,-0.807220,-0.678707,-0.000000},
                                {-0.000004,-0.153594,-0.807223,-0.598735,-0.000000},
                                {-0.000004,0.678005,0.682002,1.740000,0.000000},
                                {-0.000004,0.797084,0.658329,1.647663,0.000000},
                                {-0.000004,-0.015269,-1.245315,-0.337876,-0.000000},
                                {-0.315552,0.919449,0.439893,1.744200,-0.000000},
                                {-0.000004,-0.003964,-1.296847,-0.383750,0.000000},
                                {0.420705,0.895237,0.439897,1.717851,0.000000}
                              }; 
#endif

#ifdef REELYOUBOT_APPLICATION   


double ArmPositionReady[5]= {302335,126052,181068,23004,81462} ; 
double ArmPositionAppPick[5]={302335.000000,21877.000000,189136.000000,1522.000000,81462.000000};
double ArmPositionPick[5]={302335,10924,184142 ,1522 ,81462} ;                         
double ArmPositionAppDrop[5]={302335.000000,216485.000000,119579.000000,144529.000000,66146.000000};
double ArmPositionDrop[5]={302335.000000,225987.000000,115578.000000,138531.000000,66146.000000};


#endif                     
                      
#endif