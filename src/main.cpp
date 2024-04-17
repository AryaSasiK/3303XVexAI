/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       robol                                                     */
/*    Created:      4/16/2024, 5:55:53 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

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

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  Intake.setVelocity(100, pct); 
  Brain.Screen.print("Code is running (no ai)");
  Drivetrain.setDriveVelocity(100, pct);
  //Drivetrain.drive(fwd);
  wait(10, sec);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    leftDrive.spin(fwd);
  rightDrive.spin(fwd);
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
