/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----
#include "ai_functions.h"

using namespace vex;

brain Brain;
// Robot configuration code.
motor left1 = motor(PORT13, ratio6_1, false);
motor left2 = motor(PORT8, ratio6_1, true);
motor left3 = motor(PORT11, ratio6_1, true);
motor left4 = motor(PORT7, ratio6_1, false);

motor right1 = motor(PORT20, ratio6_1, true);
motor right2 = motor(PORT16, ratio6_1, true);
motor right3 = motor(PORT12, ratio6_1, false);
motor right4 = motor(PORT10, ratio6_1, false);

motor_group leftDrive = motor_group(left1, left2, left3, left4);
motor_group rightDrive = motor_group(right1, right2, right3, right4);

gps GPS = gps(PORT1, 0, 0, distanceUnits::in, 0, turnType::right);
smartdrive Drivetrain = smartdrive(leftDrive, rightDrive, GPS, 319.19, 320, 40, mm, 0.6);
motor Arm = motor(PORT3, ratio18_1, false);

motor IntakeRight = motor(PORT5, ratio18_1, true);
motor IntakeLeft = motor(PORT6, ratio18_1, false);
motor_group Intake = motor_group(IntakeRight, IntakeLeft);


// A global instance of competition
competition Competition;

// create instance of jetson class to receive location and other
// data from the Jetson nano
//
ai::jetson  jetson_comms;

/*----------------------------------------------------------------------------*/
// Create a robot_link on PORT1 using the unique name robot_32456_1
// The unique name should probably incorporate the team number
// and be at least 12 characters so as to generate a good hash
//
// The Demo is symetrical, we send the same data and display the same status on both
// manager and worker robots
// Comment out the following definition to build for the worker robot
#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::manager );
#else
#pragma message("building for the worker")
ai::robot_link       link( PORT11, "robot_32456_1", linkType::worker );
#endif

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) {
  // Calibrate GPS Sensor
  GPS.calibrate();
  // Optional wait to allow for calibration
  wait(1,sec);
  // Finds and moves robot to position of closest green triball
  
  rightDrive.setVelocity(100, pct);
  leftDrive.setVelocity(100, pct);

  wait(100, seconds);
  //getObject();
  // Intakes the ball
  /*
  double rot = Arm.position(rotationUnits::deg);
  intake(rot - 100, 1);
  
  // Moves to position in front of blue goal
  wait(1,sec);
  goToGoal(0);
  // Scores tri-ball in blue goal
  dump(0);
  */

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


void auto_Interaction(void) {
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

void autonomousMain(void) {
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

void usercontrol(void) {
  while (true) {
    wait(4, timeUnits::msec);
  }
}

int main() {

  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;

  // Run at about 15Hz
  int32_t loop_time = 33;

  // start the status update display
  thread t1(dashboardTask);

  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomousMain);
  Competition.drivercontrol(usercontrol);

  // print through the controller to the terminal (vexos 1.0.12 is needed)
  // As USB is tied up with Jetson communications we cannot use
  // printf for debug.  If the controller is connected
  // then this can be used as a direct connection to USB on the controller
  // when using VEXcode.
  //
  //FILE *fp = fopen("/dev/serial2","wb");
  this_thread::sleep_for(loop_time);

  while(1) {
      // get last map data
      jetson_comms.get_data( &local_map );

      // set our location to be sent to partner robot
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );

      // fprintf(fp, "%.2f %.2f %.2f\n", local_map.pos.x, local_map.pos.y, local_map.pos.az)

      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();

      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}