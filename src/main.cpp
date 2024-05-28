/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "robot-config.h"
using namespace std;
using namespace vex;

//Realsense Offsets (15in, 24in)
// X(0.25in,0in), Y(-4.25in,12in), Z(9.125in,11in), Heading(0,0), Elevation(0,0)

// GPS Offsets (15in, 24in)
// X(0,0), Y(-6.5in,8in), Z(9.875,11in), Heading(180,180)
/////********************************************************/////

/////*********STOP*************STOP*************STOP*********/////
/////**DONT FORGET TO DEFINE OR COMMENT IN "robot-config.h"**/////
/////********************************************************/////
/////**********Red Side = true || Blue Side = false**********/////
/////********************************************************/////
#define  Alliance  false

#if(Alliance)
#pragma message("Selected Red Side")
#else
#pragma message("Selected Blue Side")
#endif


#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT7, "24in 3303X", linkType::manager );
//24in Robot Specific Objects
motor leftDriveA = motor(PORT9, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT7, ratio6_1, true);   
motor leftDriveD = motor(PORT8, ratio6_1, true);
motor rightDriveA = motor(PORT3, ratio6_1, false);
motor rightDriveB = motor(PORT4, ratio6_1, false);
motor rightDriveC = motor(PORT1, ratio6_1, false);
motor rightDriveD = motor(PORT2, ratio6_1, false);
motor Catapult = motor(PORT11,ratio36_1,true);
rotation CatapultEnc = rotation(PORT16);
limit CatapultLimit = limit(Brain.ThreeWirePort.D);
optical Balldetect = optical(PORT14);
motor Intake = motor(PORT12, ratio6_1, true);
motor HangA = motor(PORT13, ratio36_1, false);
motor HangB = motor(PORT14, ratio36_1, true);
rotation HangEnc = rotation(PORT5);
gps GPS = gps(PORT20, 0.0, -163.4, mm, 180);
const int32_t InertialPort = PORT19;
double Robot_x_Offset = 25.4;
double Intake_Offset = 15;
double wheel_size = 3.15;
digital_out R_Wing = digital_out(Brain.ThreeWirePort.C);
digital_out L_Wing = digital_out(Brain.ThreeWirePort.E);
digital_out  Hang_Ratchet = digital_out(Brain.ThreeWirePort.H);





#else
#pragma message("building for the worker")
ai::robot_link       link(PORT7, "15in 3303X", linkType::worker );
//15in Robot Specific Objects
motor leftDriveA = motor(PORT20, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT19, ratio6_1, false);   
motor leftDriveD = motor(PORT9, ratio6_1, false);   
motor rightDriveA = motor(PORT12, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, false);
motor rightDriveC = motor(PORT7, ratio6_1, true);
motor rightDriveD = motor(PORT4, ratio6_1, true);
optical Balldetect = optical(PORT14);
motor Intake = motor(PORT11, ratio6_1, true);
motor HangA = motor(PORT15, ratio36_1, false);
motor HangB = motor(PORT13, ratio36_1, true);
gps GPS = gps(PORT3, 0.0, -146, mm, 180);
const int32_t InertialPort = PORT16;
double Robot_x_Offset = 20;
double Intake_Offset = 8.0;
double wheel_size = 3.25;
digital_out R_Wing = digital_out(Brain.ThreeWirePort.C);
digital_out L_Wing = digital_out(Brain.ThreeWirePort.E);
digital_out  Hang_Ratchet = digital_out(Brain.ThreeWirePort.H);
#endif


controller Controller1 = controller(primary);

competition Competition;
ai::jetson  jetson_comms;// create instance of jetson class to receive location and other

//Universal Objects (Do not comment out)
// Red Side = true || Blue Side = false
Field field(Alliance,Robot_x_Offset,Intake_Offset);
FILE *fp = fopen("/dev/serial2","wb");
brain Brain;
timer Match = timer();
motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC, leftDriveD);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC,rightDriveD);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, wheel_size, 0.6, 360);
motor_group Hang = motor_group(HangA, HangB);
bool wait2hang = true;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tuned_constants()
{
  #if defined(MANAGER_ROBOT)

  Chassis.set_turn_constants(12, 0.18, 0.018, 1.25, 15);
  Chassis.set_drive_constants(12, 1.4, 0, 16, 0);
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_swing_constants(12, .3, .001, 2, 15);
  Chassis.set_drive_exit_conditions(1.5, 300, 5000);
  Chassis.set_turn_exit_conditions(1, 300, 3000);
  Chassis.set_swing_exit_conditions(1, 300, 3000);
  #else
  Chassis.set_drive_constants(12, 1.5, 0, 10, 0);
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_turn_constants(12, 0.25, 0.0005, 1.25, 15);
  Chassis.set_swing_constants(12, .3, .001, 2, 15);
  Chassis.set_drive_exit_conditions(1.5, 300, 1500);
  Chassis.set_turn_exit_conditions(1, 300, 1000);
  Chassis.set_swing_exit_conditions(1, 300, 3000);
  #endif
}

void pre_auton(void) 
{
  
  Balldetect.objectDetectThreshold(65);
  Intake.setVelocity(100,pct);
  tuned_constants();
  Chassis.Gyro.calibrate();
  while (Chassis.Gyro.isCalibrating()) 
  {
    wait(25, msec);
  }
  GPS.calibrate();
  while (GPS.isCalibrating()) 
  {
    wait(25, msec);
  }
  wait(50, msec);
  Brain.Screen.clearScreen();
}

bool Controller1LeftShoulderControlMotorsStopped = true;
void usercontrol(void) 
{
  Intake.setVelocity(100,pct);
  while(1)
  {
    Chassis.control_arcade();

    if (Controller1.ButtonL1.pressing()) 
    {
      Intake.spin(vex::directionType::fwd);
      Controller1LeftShoulderControlMotorsStopped = false;
    } 
    else if (Controller1.ButtonL2.pressing()) 
    {
      Intake.spin(vex::directionType::rev);
      Controller1LeftShoulderControlMotorsStopped = false;
    } 
    else if (!Controller1LeftShoulderControlMotorsStopped) 
    {
      Intake.stop();
      Controller1LeftShoulderControlMotorsStopped = true;
    }
    wait(20,msec);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void testing_tuning(void)
{
  GetMatchLoad();
  Move2Drop_Pos();
  //Chassis.set_heading(180);
  // Chassis.drive_distance(48,180);
  //Chassis.turn_max_voltage = 12 ; 
  //Chassis.turn_to_angle(0);
  // getObject(false,false);
  // ScoreBall();
  // moveToPosition(0,158,90,false,75,75);

  // while(true)
  // {
  //   if(getObject(true,false))
  //   {
  //     wait(500,msec);
  //     ScoreBall();
  //   }
  // }
} 

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



int endGameTimer()
{
  int time2hang = 55;
  while(Match.time(vex::timeUnits::sec)< time2hang)
  {
    task::sleep(1000);
  }
  wait2hang = false;

  return 0;
}

void auto_Isolation(void) 
{ 
  #if defined(MANAGER_ROBOT)

  #else
  while (true)
  {
    if(getObject(true,true))
    {
      ScoreBall();
    }
  }
  #endif
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Interaction(void) 
{
  Match.clear();
  task endgame(endGameTimer);

  #if defined(MANAGER_ROBOT)

  #else
  while (wait2hang)
  {
    if(getObject(true,false))
    {
      ScoreBall();
    }
  }
  #endif
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;
void autonomousMain(void) 
{
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}


int main() {

 
  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;
  // Run at about 15Hz
  int32_t loop_time = 33;
  // start the status update display
  thread t1(dashboardTask);
  pre_auton(); 
  // Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(testing_tuning);
  //Match.event(testing_tuning,10);
  // Competition.autonomous(autonomousMain);
  this_thread::sleep_for(loop_time);
  int counter = 0 ;
  while(1) 
  {

      
      jetson_comms.get_data( &local_map ); // get last map data
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );// set our location to be sent to partner robot

      counter += 1 ;
      if (counter > 15)
      {
        //fprintf(fp,"\rTimer Value: %.1f\n",Match.time(vex::timeUnits::sec));
        //fprintf(fp,"\rLocal Map Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",local_map.pos.az,local_map.pos.x*100,local_map.pos.y*100);
        fprintf(fp,"\rGPS Pos Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));
        counter = 0 ;
      }
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();
      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}