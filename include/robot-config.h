#include "vex.h"
using namespace vex;

extern brain Brain;
extern Drive Chassis;  
extern gps GPS;
extern motor Intake; 
extern motor_group Hang;
extern optical Balldetect;
extern digital_out R_Wing;
extern digital_out L_Wing;
extern digital_out Hang_Ratchet;
extern controller Controller;
extern FILE* fp;

#define  MANAGER_ROBOT  1

#if defined(MANAGER_ROBOT)
extern motor Catapult;
extern rotation CatapultEnc;
extern rotation HangEnc;
extern limit CatapultLimit;

#endif

