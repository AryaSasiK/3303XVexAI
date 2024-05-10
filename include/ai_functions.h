/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Header for AI robot movement functions                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <vex.h>
#include <robot-config.h>


// Calculates the distance to a given target (x, y)
float distanceTo(double target_x, double target_y,vex::distanceUnits unit);

// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta, int Dspeed, int Tspeed, bool frontfacing);

// Finds a target object based on the specified type
DETECTION_OBJECT findTarget();

// Retrieves an object (e.g. from the ground or a dispenser)
void getObject();

// Turns the robot to a specific angle with given tolerance and speed
void turnTo(double angle, int speed, bool frontfacing);

void ScoreBall();
