/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
using namespace std;
using namespace vex;

//Realsense Offsets (15in, 24in)
// X(0.25in,0in), Y(-4.25in,12in), Z(9.125in,11in), Heading(0,0), Elevation(0,0)

// GPS Offsets (15in, 24in)
// X(0,0), Y(-6.5in,8in), Z(9.875,11in), Heading(180,180)

#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT21, "24in 3303X", linkType::manager );
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
optical Balldetect = optical(PORT14);
motor Intake = motor(PORT11, ratio6_1, true);
gps GPS = gps(PORT20, 0.0, 203.2, mm, 180);
const int32_t InertialPort = PORT18;
const int32_t HangAPort = PORT14;
const int32_t HangBPort = PORT13;
double Robot_x_Offset = 25.4;

#else
#pragma message("building for the worker")
ai::robot_link       link( PORT21, "15in 3303X", linkType::worker );
//15in Robot Specific Objects
motor leftDriveA = motor(PORT20, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT19, ratio6_1, false);   
motor leftDriveD = motor(PORT9, ratio6_1, false);   
motor rightDriveA = motor(PORT12, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, false);
motor rightDriveC = motor(PORT7, ratio6_1, true);
motor rightDriveD = motor(PORT4, ratio6_1, true);
motor Intake = motor(PORT11, ratio6_1, true);
gps GPS = gps(PORT3, 0.0, -146, mm, 180);
const int32_t InertialPort = PORT16;
const int32_t HangAPort = PORT15;
const int32_t HangBPort = PORT13;
double Robot_x_Offset = 1;

#endif


Field field(vex::color::blue,Robot_x_Offset);
FILE *fp = fopen("/dev/serial2","wb");
brain Brain;
controller Controller1 = controller(primary);
competition Competition;
ai::jetson  jetson_comms;// create instance of jetson class to receive location and other

//Universal Objects (Do not comment out)

motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC, leftDriveD);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC,rightDriveD);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, 3.125, 0.6, 360);
motor HangA = motor(HangAPort, ratio36_1, false);
motor HangB = motor(HangBPort, ratio36_1, true);
motor_group Hang = motor_group(HangA, HangB);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tuned_constants()
{
  Chassis.set_drive_constants(12, 1.5, 0, 10, 0);
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_turn_constants(12, 0.25, 0.0005, 1.25, 15);//Tuned
  Chassis.set_swing_constants(12, .3, .001, 2, 15);
  Chassis.set_drive_exit_conditions(1.5, 300, 5000);
  Chassis.set_turn_exit_conditions(1, 300, 3000);
  Chassis.set_swing_exit_conditions(1, 300, 3000);
}

void pre_auton(void) 
{
  tuned_constants();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  Chassis.Gyro.calibrate();
  Brain.Screen.print("Calibrating Inertial for Chassis");
  Brain.Screen.setCursor(3, 1);
  // wait for the Inertial calibration process to finish
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
   while(true)
  {
    if(getObject())
    {
      ScoreBall();
      wait(2, sec);
    }
  }
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

void auto_Isolation(void) 
{
//  Chassis.turn_to_angle(90);
 Chassis.drive_distance(50);
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
  Competition.autonomous(auto_Isolation);
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
        //testing_tuning();  
        //fprintf(fp,"\nPositional Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",local_map.pos.az,local_map.pos.x*100,local_map.pos.y*100);
        //fprintf(fp,"\nGPS Positional Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));
        counter = 0 ;
      }
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();
      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}