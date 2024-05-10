/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Copyright (c) Innovation First 2023 All rights reserved.                */
/*    Licensed under the MIT license.                                         */
/*                                                                            */
/*    Module:     ai_functions.cpp                                            */
/*    Author:     VEX Robotics Inc.                                           */
/*    Created:    11 August 2023                                              */
/*    Description:  Helper movement functions for VEX AI program              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// #include "vex.h"
#include "ai_functions.h"
#include "field.h"
#include <string>
#include <iostream>

using namespace std;
Field field(red);
// Calculates the distance to the coordinates from the current robot position

float distanceTo(double target_x, double target_y, vex::distanceUnits unit = vex::distanceUnits::in)
{
    float distance = sqrt(pow((target_x - GPS.xPosition(vex::distanceUnits::cm)), 2) + pow((target_y - GPS.yPosition(vex::distanceUnits::cm)), 2));
    if (unit == vex::distanceUnits::mm)
        distance = distance / 10;
    if (unit == vex::distanceUnits::in)
        distance = distance / 2.54;

    return distance;
}

// Calculates the bearing to drive to the target coordinates in a straight line aligned with global coordinate/heading system.
double calculateBearing(double currX, double currY, double targetX, double targetY)
{
    // Calculate the difference in coordinates
    double dx = targetX - currX;
    double dy = targetY - currY;

    // Calculate the bearing in radians
    double bearing_rad = atan2(dy, dx);

    // Convert to degrees
    double bearing_deg = bearing_rad * 180 / M_PI;

    // Normalize to the range 0 to 360
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }

    // Convert from mathematical to navigation coordinates
    bearing_deg = fmod(90 - bearing_deg, 360);
    if (bearing_deg < 0)
    {
        bearing_deg += 360;
    }

    // Normalize to the range [-180, 180]
    // This allows turnTo to make the smallest turn, whether going forward or backwards.
    bearing_deg = fmod(bearing_deg, 360);
    if (bearing_deg > 180)
    {
        bearing_deg -= 360;
    }
    else if (bearing_deg < -180)
    {
        bearing_deg += 360;
    }
    // fprintf(fp,"Target bearing:%.2f Degrees\n",bearing_deg);
    return bearing_deg;
}

// Turns the robot to face the angle specified, taking into account a tolerance and speed of turn.
void turnTo(double angle, int speed = 75, bool frontfacing = true)
{
    float voltage = (speed * 0.12);
    Chassis.set_heading(GPS.heading(deg));
    if (frontfacing)
        Chassis.turn_to_angle(angle, voltage);
    else
        Chassis.turn_to_angle(180 + angle, voltage);
}
void moveToPoint(Point *Target, int Dspeed = 75, int Tspeed = 75, bool frontfacing = true)
{
    float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), Target->Xcord, Target->Ycord);
    turnTo(intialHeading, Tspeed, frontfacing);
    float distance = distanceTo(Target->Xcord, Target->Ycord);
    float voltage = (Dspeed * .12);
    if (frontfacing)
    {
        Chassis.drive_distance(distance, intialHeading, voltage, voltage);
    }
    else
    {
        Chassis.drive_distance(-distance, 180 + intialHeading, voltage, voltage);
    }
}
// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, int Dspeed = 75, int Tspeed = 75, bool frontfacing = true)
{

    float targetThreshold = 7; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrivedtoTarget = false;
    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm));

    if (!field.Check_Barrier_Intersects(&CurrentPoint, &Target))
    {
        while (!arrivedtoTarget)
        {
            float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), target_x, target_y);
            turnTo(intialHeading, Tspeed, frontfacing);
            float distance = distanceTo(target_x, target_y);
            float voltage = (Dspeed * .12);
            if (frontfacing)
            {
                Chassis.drive_distance(distance, intialHeading, voltage, voltage);
            }
            else
            {
                Chassis.drive_distance(-distance, 180 + intialHeading, voltage, voltage);
            }

            float dist = (target_x - GPS.xPosition(distanceUnits::cm)) * (target_x - GPS.xPosition(distanceUnits::cm)) + (target_y - GPS.xPosition(distanceUnits::cm)) * (target_y - GPS.xPosition(distanceUnits::cm));
            if (dist <= targetThreshold * targetThreshold)
            {
                arrivedtoTarget = true;
            }
        }
    }
    else
    {
        Path Path2Target = field.Create_Path_to_Target(&Target);
        for (int i = 0; i < Path2Target.PathPoints.size(); i++)
        {
            moveToPoint(Path2Target.PathPoints[i], Dspeed, Tspeed, frontfacing);
        }
    }
    if (target_theta != -1)
    {
        turnTo(target_theta);
    }
}

// Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget()
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    for (int i = 0; i < local_map.detectionCount; i++)
    {
        if(!field.In_Goal_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
        {
            double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
            if (distance < lowestDist)
            {
                if(field.Side == vex::color::red)
                {
                    if(local_map.detections[i].classID == 0 || 1)
                    {
                        target = local_map.detections[i];
                        lowestDist = distance;
                    }
                }
                else if(field.Side == vex::color::blue)
                {
                    if(local_map.detections[i].classID == 0 || 2)
                    {
                        target = local_map.detections[i];
                        lowestDist = distance;
                    }
                }
                else
                {
                    target = local_map.detections[i];
                    lowestDist = distance;
                }
            }
        }
    }
    return target;
}

// Function to retrieve an object based on detection
void getObject()
{
    Intake.setVelocity(100,vex::pct);
    Balldetect.objectDetectThreshold(100);
    bool HoldingBall = false; 
    float turn_step = 45;
    DETECTION_OBJECT Triball = findTarget();
    while(!HoldingBall)
    {
        
        while(Triball.mapLocation.x && Triball.mapLocation.y == 0)
        {
            Chassis.turn_to_angle(GPS.heading()+turn_step);
            Triball = findTarget();
        }
    
        Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        moveToPosition(Triball.mapLocation.x * 100, Triball.mapLocation.y * 100);
        if(Balldetect.color() == vex::color::red || vex::color::green || vex::color::blue)
            if(Balldetect.isNearObject())
            {
                HoldingBall = true;
            }
        Triball = findTarget();
    }
}

void ScoreBall()
{
    if (GPS.xPosition(distanceUnits::cm) < 0.00) // we are on right side(Blue Goal)
    {
        moveToPosition(-61.35, 0.00, 90);
    }
    else if (GPS.xPosition(distanceUnits::cm) > 0.00) // we are on right side(Blue Goal)
    {
        moveToPosition(61.35, 0.00, 270);
    }
    Intake.spin(reverse);
    Chassis.drive_distance(25);
    Intake.stop(coast);
    Chassis.drive_distance(-15);
}
