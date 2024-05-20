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
using namespace vex;
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


void moveToPoint(Point* Target, bool frontfacing)
{   
    float ThresholdRad = 25.4; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrivedtoTarget = false;
    // while(!arrivedtoTarget)
    // {
    //     //Check to see if we have arrived to target 
    //     float threshold = (GPS.xPosition(distanceUnits::cm) - Target->Xcord) * (GPS.xPosition(distanceUnits::cm) - Target->Xcord) + (GPS.yPosition(distanceUnits::cm) -Target->Ycord ) * (GPS.yPosition(distanceUnits::cm) - Target->Ycord );
    //     if (threshold <= pow(ThresholdRad, 2))
    //     {
    //             arrivedtoTarget = true;
    //     }
        //Turn Function
        float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), Target->Xcord, Target->Ycord);
        if (!frontfacing)
        {
            intialHeading = intialHeading + 180;
        }
        Chassis.set_heading(GPS.heading(deg));
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        if (!frontfacing)
        {
            distance = distance * -1;
        }
        Chassis.drive_distance(distance);
    // }

}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, bool frontfacing = true, int Dspeed = 75, int Tspeed = 75)
{
    Chassis.drive_max_voltage = Dspeed * 0.12;
    Chassis.turn_max_voltage = Tspeed * 0.12;

    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm));

    if (!field.Check_Barrier_Intersects(CurrentPoint, Target,true))
    {
        fprintf(fp,"No Barrier Intersection found moving to point\n");
        moveToPoint(&Target,frontfacing);
    }
    else
    {
        fprintf(fp,"Barrier Intersection found! Creating Path to Target\n");
        Path Path2Follow = field.Create_Path_to_Target(Target);
        Path2Follow.calcPathLength();
        fprintf(fp,"\nPath Length: %.2f || Number of Points in Path %i || Start of Path: (%.2f, %.2f)", Path2Follow.pathlength, Path2Follow.PathPoints.size(), Path2Follow.PathPoints[0]->Xcord, Path2Follow.PathPoints[0]->Ycord);
        // for (int i = 0; i < Path2Follow.PathPoints.size(); i++)
        // {
        //     fprintf(fp,"-> (%.2f, %.2f)",Path2Follow.PathPoints[i]->Xcord, Path2Follow.PathPoints[i]->Xcord);
        // }
        for (int i = 1; i < Path2Follow.PathPoints.size() -1 ; i++)
        {
            moveToPoint(Path2Follow.PathPoints[i], frontfacing);
        }
    }
    if (target_theta != -1)
    {
        Chassis.turn_to_angle(target_theta);
    }
}

bool On_Goal_Side(double target_x, bool checkside)
{
    if(checkside)
    {
        if(!field.Blue_Side)
        {
            if(target_x < 0)
            {
                return false;
            }
        }
        else if(!field.Red_Side)
        {
            if(target_x > 0)
            {
                return false;
            }
        }
    }
    return true; 
}

//Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget()
{
    DETECTION_OBJECT target;
    target.mapLocation.x = 0;
    target.mapLocation.y = 0;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    for (int i = 0; i < local_map.detectionCount; i++)
    {
        if(!field.In_Goal_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y) && !field.In_MatchLoad_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
        {
            if(field.Blue_Side && On_Goal_Side)
            {
                if(local_map.detections[i].classID == 0)
                {
                    double distance = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                    if (distance < lowestDist)
                    {
                        target = local_map.detections[i];
                        lowestDist = distance;     
                    }
                }
            }
            if(field.Red_Side && On_Goal_Side)
            {
                if(local_map.detections[i].classID == 0   )
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

bool CheckBallColor()
{
    if(Balldetect.color() == red)
        return true;
    else if(Balldetect.color() == blue)
        return true;
    else if(Balldetect.color() == green)
        return true;

    return false;
}

// Function to retrieve an object based on detection
bool getObject()
{
    bool HoldingBall = false; 
    float turn_step = 45;
    DETECTION_OBJECT Triball;
    Triball.mapLocation.x = 0; 
    Triball.mapLocation.y = 0;
    Balldetect.objectDetectThreshold(65);
    Intake.setVelocity(100,pct);

    while(!HoldingBall)
    {
        if(Balldetect.isNearObject() && CheckBallColor())
        {
            fprintf(fp,"The robot is holding a Triball");
            Intake.stop(hold);
            HoldingBall = true;
        }
      
        int turn_iter = 0;
        while(Triball.mapLocation.x && Triball.mapLocation.y == 0.00)
        {
        
            fprintf(fp,"Seanning for ball....");
            Chassis.turn_max_voltage = 3;
            Chassis.turn_to_angle(GPS.heading()+turn_step);
            wait(1,sec);
            Triball = findTarget();
            turn_iter = turn_iter + 1 ;
            if(turn_iter > 7)
            {
                return HoldingBall;
            }
        }


        if(Triball.mapLocation.x && Triball.mapLocation.y != 0.00)
        {
            fprintf(fp,"Found Triball! || Triballl Location (%.2f, %.2f)", Triball.mapLocation.x, Triball.mapLocation.y);
            Intake.spin(vex::directionType::fwd);
            moveToPosition(Triball.mapLocation.x * 100, Triball.mapLocation.y * 100);
            wait(750,msec);
        }
    
        Triball = findTarget();
    }

    return HoldingBall;
}

void ScoreBall()
{  
    if(field.Blue_Side)
    {
        moveToPosition(-61.35,0.0,270.0);
    }
    if(field.Red_Side)
    {
        moveToPosition(61.35,0.0,90.0);
    }
    Intake.setVelocity(100,pct);
    Intake.spin(vex::directionType::rev);
    wait(500,msec);
    Chassis.drive_distance(15);
    Intake.stop(coast);
    Chassis.drive_distance(-15);
    Chassis.turn_to_angle(90);
}
