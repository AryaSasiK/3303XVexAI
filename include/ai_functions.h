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

#include <field.h>
extern Field field;
// Calculates the distance to a given target (x, y)
float distanceTo(double target_x, double target_y,vex::distanceUnits unit);
double calculateBearing(double currX, double currY, double targetX, double targetY);
void moveToPoint(Point* Target, bool FrontFacing);
void MovetoBall(Point* Target);
// Moves the robot to a specified position and orientation
void moveToPosition(double target_x, double target_y, double target_theta, bool frontfacing, int Dspeed, int Tspeed);
// Finds a target object based on the specified type
DETECTION_OBJECT findTarget(bool CheckSide, bool CheckIso);
bool CheckBallColor();
// Retrieves an object (e.g. from the ground or a dispenser)
bool getObject(bool CheckSide, bool CheckIso);
void ScoreBall();
#if defined(MANAGER_ROBOT)
bool GetMatchLoad();
void Move2Drop_Pos();
void ThrowBall();
void BlockIntake();
#endif
