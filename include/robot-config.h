#include "vex.h"
using namespace vex;

extern brain Brain;
extern Drive Chassis;  
extern gps GPS;
extern motor Intake; 
extern motor_group Hang;
extern optical Balldetect;
extern controller Controller;
extern FILE* fp;

//#define  MANAGER_ROBOT  1

#if defined(MANAGER_ROBOT)
extern motor Catapult;
#endif

