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

#include "ai_functions.h"
using namespace std;
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
    {
        Chassis.turn_to_angle(angle, voltage);
    }
    else
    {
        Chassis.turn_to_angle(180 + angle, voltage);
    }
}

void moveToPoint(Point* Target, bool frontfacing, int Dspeed, int Tspeed)
{   float targetThreshold = 12.7; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrivedtoTarget = false;
    while(!arrivedtoTarget)
    {
        float dist = (GPS.xPosition(distanceUnits::cm) - Target->Xcord) * (GPS.xPosition(distanceUnits::cm) - Target->Xcord) + (GPS.yPosition(distanceUnits::cm) -Target->Ycord ) * (GPS.xPosition(distanceUnits::cm) - Target->Ycord );
        if (dist <= (targetThreshold * targetThreshold))
        {
                arrivedtoTarget = true;
        }
        float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), Target->Xcord, Target->Ycord);
        turnTo(intialHeading, Tspeed, frontfacing);
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        float voltage = (Dspeed * .12);
        if(frontfacing)
        {
            Chassis.drive_distance(distance, intialHeading, voltage, voltage);
        }
        else
        {
            Chassis.drive_distance(-distance, 180 + intialHeading, voltage, voltage);
        }
    
    }

}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, bool frontfacing = true, int Dspeed = 75, int Tspeed = 75)
{
    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm));
    //Point CurrentPoint(-40, -40);
    if (!field.Check_Barrier_Intersects(CurrentPoint, Target))
    {
        fprintf(fp,"No Barrier Intersection found moving to point\n");
        moveToPoint(&Target,frontfacing,Dspeed,Tspeed);
    }
    else
    {
        fprintf(fp,"Barrier Intersection found! Creating Path to Target\n");
        Path Path2Follow = field.Create_Path_to_Target(Target);
        for (int i = 0; i < Path2Follow.PathPoints.size(); i++)
        {
            moveToPoint(Path2Follow.PathPoints[i], frontfacing, Dspeed, Tspeed);
        }
    }
    if (target_theta != -1)
    {
        turnTo(target_theta);
    }
}
bool OnSide(double target_x, bool checkside)
{
    if(checkside)
    {
        if(field.Side == vex::color::red)
        {
            if(target_x < 0)
                return false;
        }
        else if(field.Side == vex::color::blue)
        {
            if(target_x > 0)
                return false;
        }
    }
    return true; 

}

//Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget()
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    bool goal_check;

    for (int i = 0; i < local_map.detectionCount; i++)
    {
        if(!field.In_Goal_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
        {
            if(field.Blue_Side)
            {
                if(local_map.detections[i].classID == 0 || local_map.detections[i].classID == 2)
                {
                    double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                    if (distance < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = distance;     
                    }
                }
            }
            if(field.Red_Side)
            {
                if(local_map.detections[i].classID == 0 || local_map.detections[i].classID == 1)
                {
                    double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                    if (distance < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = distance;     
                    }
                }
            }
        }  
    }
    return target;
}


// Function to retrieve an object based on detection
bool getObject()
{
    bool HoldingBall = false; 
    float turn_step = 45;
    DETECTION_OBJECT Triball;
    int turn_iter;
    int hold_iter = 0 ;
    while(!HoldingBall)
    {
        Triball = findTarget();
        turn_iter = 0;
        while(Triball.mapLocation.x && Triball.mapLocation.y == 0)
        {
            Chassis.turn_to_angle(GPS.heading()+turn_step);
            Triball = findTarget();
            turn_iter = turn_iter + 1 ;
        }
        Intake.spin(vex::directionType::fwd,100,vex::velocityUnits::pct);
        moveToPosition(Triball.mapLocation.x * 100, Triball.mapLocation.y * 100,true);
        if(Balldetect.isNearObject())
        {
            Intake.stop(hold);
            return true;

        }
    }
    return false;

}

void ScoreBall()
{  
    moveToPosition(61.35,0.00,90);
   
    Intake.spin(vex::directionType::rev,100,vex::velocityUnits::pct);
    Chassis.drive_distance(25);
    Intake.stop(coast);
    Chassis.drive_distance(-15);
}
