/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "field.h"



using namespace std;
using namespace vex;


//#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::worker );
#endif

//Realsense Offsets (15in, 24in)
// X(0.25in,0in), Y(-4.25in,12in), Z(9.125in,11in), Heading(0,0), Elevation(0,0)

// GPS Offsets (15in,24in)
// X(0,0), Y(-6.5in,8in), Z(9.875,11in), Heading(180,180)


// ---- START CONFIGURED DEVICES ----

//24in Robot Specific Objects
// motor leftDriveA = motor(PORT9, ratio6_1, true);  
// motor leftDriveB = motor(PORT10, ratio6_1, true);   
// motor leftDriveC = motor(PORT7, ratio6_1, true);   
// motor leftDriveD = motor(PORT8, ratio6_1, true);

// motor rightDriveA = motor(PORT3, ratio6_1, false);
// motor rightDriveB = motor(PORT4, ratio6_1, false);
// motor rightDriveC = motor(PORT1, ratio6_1, false);
// motor rightDriveD = motor(PORT2, ratio6_1, false);
// const int32_t InertialPort = PORT19;
// const int32_t HangAPort = PORT14;
// const int32_t HangBPort = PORT13;
// const int32_t IntakePort = PORT12;
// const int32_t GPSPort = PORT20;
// double GPS_y_Offset = 203.2;
// motor Catapult = motor(PORT11,ratio36_1,true);
// double Robot_x_Offset = 25.4;


//15in Robot Specific Objects
motor leftDriveA = motor(PORT20, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT19, ratio6_1, false);   
motor leftDriveD = motor(PORT9, ratio6_1, false);   
motor rightDriveA = motor(PORT12, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, false);
motor rightDriveC = motor(PORT7, ratio6_1, true);
motor rightDriveD = motor(PORT4, ratio6_1, true);
const int32_t InertialPort = PORT16;
const int32_t HangAPort = PORT16;
const int32_t HangBPort = PORT16;
const int32_t IntakePort = PORT11;
const int32_t GPSPort = PORT3;
double GPS_y_Offset = -146.0;
double Robot_x_Offset = 19;


//Universal Objects (Do not comment out)
optical Balldetect = optical(PORT14);
motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC, leftDriveD);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC,rightDriveD);
Drive Chassis(ZERO_TRACKER_NO_ODOM,LeftDriveSmart,RightDriveSmart,InertialPort,3.125,0.6,360,PORT1,-PORT2,PORT3,-PORT4,3,2.75,-2,1,-2.75,5.5);
gps GPS = gps(GPSPort, 0.0, GPS_y_Offset, mm, 180);
motor HangA = motor(HangAPort, ratio36_1, false);
motor HangB = motor(HangBPort, ratio36_1, true);
motor_group Hang = motor_group(HangA, HangB);
motor Intake = motor(IntakePort, ratio6_1, true);
Field field(purple,Robot_x_Offset);
FILE *fp = fopen("/dev/serial2","wb");

brain Brain;
controller Controller1 = controller(primary);
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
ai::jetson  jetson_comms;
/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot

// ---- END CONFIGURED DEVICES ----

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
  Balldetect.objectDetectThreshold(65);
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
 // Brain.Screen.print("Calibrating GPS for VEX AI");
  while (GPS.isCalibrating()) 
  {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  //Brain.Screen.clearScreen();
  //Brain.Screen.setCursor(1,1);

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

void testing_tuning(void)
{
  //moveToPosition(122,122,-1,true,100,100);
  //moveToPosition(-61.35,61.35,-1,true,100,100);
  //moveToPosition(95.00,150.51,-1,true,50,50);
  //getObject();
  // for(int i = 0; i < field.Path2Snap2.size(); i++)
  // {
  //   fprintf(fp, "(%.2f,%.2f) -> ", field.Path2Snap2[i]->Xcord, field.Path2Snap2[i]->Xcord);
  // }
  //Print_Path(&test);
  
  //fprintf(fp, "(%.2f,%.2f) -> ", field.Path2Snap2[0]->Xcord, field.Path2Snap2[0]->Ycord);
  //while(true)
    //fprintf(fp,"\n\n\n\n GPS Positional Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));

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

  // Functions needed: evaluate which ball detected is target, go to target (x,y), intake ball, dump ball, 
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
  Competition.autonomous(auto_Isolation);
  //Competition.autonomous(autonomousMain);
  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");

  this_thread::sleep_for(loop_time);
  int counter = 0 ;
  while(1) 
  {

      // get last map data
      jetson_comms.get_data( &local_map );
      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

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