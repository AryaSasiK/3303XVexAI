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
    
    if(unit == vex::distanceUnits::in)
        distance = distance / 2.54;
    else if(unit == vex::distanceUnits::cm)
        return distance;

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


void moveToPoint(Point* Target, bool FrontFacing = true)
{   
    float ThresholdRad = 12; // represnts the radius (cm) of the current postion if target point lies within the circle then move to postion function will end
    bool arrived2Target = false;
    
  
    while(!arrived2Target)
    {
        double X_Pos = GPS.xPosition(vex::distanceUnits::cm);
        double Y_Pos = GPS.yPosition(vex::distanceUnits::cm);
        // Check to see if we have arrived to target 
        double threshold = pow((X_Pos - Target->Xcord), 2) + pow((Y_Pos - Target->Ycord),2);
        if(threshold <= pow(ThresholdRad, 2))
        {   
                fprintf(fp,"\rRobot is within the threshold of target\n");
                break;
        }
        // Turn Function
        double intialHeading = calculateBearing(X_Pos, Y_Pos, Target->Xcord, Target->Ycord);
        double diff = fabs(GPS.heading(vex::rotationUnits::deg) - intialHeading);
        double result = (diff <= 180.0) ? diff : 360.0 - diff;

        if((result > 90))
        {
            intialHeading +=  180;
        }
        Chassis.set_heading(GPS.heading(deg));
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        if((result > 90))
        {
            distance = distance * -1;
        }
        Chassis.drive_distance(distance);
   }

}

void MovetoBall(Point* Target)
{   
        //Turn Function
        float intialHeading = calculateBearing(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm), Target->Xcord, Target->Ycord);
        Chassis.set_heading(GPS.heading(deg));
        Chassis.turn_to_angle(intialHeading);
        //Drive Function
        Chassis.desired_heading = intialHeading;
        float distance = distanceTo(Target->Xcord, Target->Ycord);
        Chassis.drive_distance(distance - field.Front_Offset);
}

// Method that moves to a given (x,y) position and a desired target theta to finish movement facing in cm
void moveToPosition(double target_x, double target_y, double target_theta = -1, bool GetBall = false, int Dspeed = 75, int Tspeed = 75)
{
    Chassis.drive_max_voltage = Dspeed * 0.12;
    Chassis.turn_max_voltage = Tspeed * 0.12;

    Point Target(target_x, target_y);
    Point CurrentPoint(GPS.xPosition(distanceUnits::cm), GPS.yPosition(distanceUnits::cm));

    if (!field.Check_Barrier_Intersects(&CurrentPoint, &Target, !GetBall))
    {
        fprintf(fp,"\rNo Barrier Intersection found moving to point\n");
        
        if(GetBall)
        {
            MovetoBall(&Target);
        }
        else
        {
            moveToPoint(&Target);
        }
    }
    else
    {
        fprintf(fp,"\rBarrier Intersection found! Creating Path to Target\n");
        Path Path2Follow = field.Create_Path_to_Target(&CurrentPoint, &Target);
        Path2Follow.calcPathLength();
        fprintf(fp,"\rPath Length: %.2f || Number of Points in Path %i || Start of Path: (%.2f, %.2f)\n", Path2Follow.pathlength, Path2Follow.PathPoints.size(), Path2Follow.PathPoints[0]->Xcord, Path2Follow.PathPoints[0]->Ycord);
        for (int i = 1; i < Path2Follow.PathPoints.size(); i++)
        {   
            fprintf(fp, "\r-> (%.2f, %.2f)\n", Path2Follow.PathPoints[i]->Xcord, Path2Follow.PathPoints[i]->Xcord);
            if(!GetBall)
            {
                moveToPoint(Path2Follow.PathPoints[i]);
            }

            else
            {
                if(i == Path2Follow.PathPoints.size() - 1)
                {
                    MovetoBall(Path2Follow.PathPoints[i]);
                }
                else
                {
                    moveToPoint(Path2Follow.PathPoints[i]);
                }
            }
        }
    }
    if (target_theta != -1)
    {
        Chassis.turn_to_angle(target_theta);
    }
}


//Function to find the target object based on type and return its record
DETECTION_OBJECT findTarget(bool CheckSide = true, bool CheckIso = false)
{
    DETECTION_OBJECT target;
    static AI_RECORD local_map;
    jetson_comms.get_data(&local_map);
    double lowestDist = 1000000;
    int Ball_Color = 0;
    if(field.Blue_Side)
        Ball_Color = 2;
    else if (field.Red_Side)
        Ball_Color = 1;
    field.Updtae_Intake_Zone();
   
    fprintf(fp,"\rNumber of balls in local map: %ld \n",local_map.detectionCount);
    for (int i = 0; i < local_map.detectionCount; i++)
    {
        if(!field.In_Goal_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y) && !field.In_MatchLoad_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
        {
            if(field.In_Offensive_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y,CheckSide))
            {
                if(field.In_Iso_Zone(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y,CheckIso))
                {
                    if(local_map.detections[i].classID == 0 || Ball_Color )
                    {
                        if(!field.Near_Intake(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y))
                        {
                            if(local_map.detections[i].probability > 0.97 && local_map.detections[i].probability <= 1) 
                            {
                                double Ball_Dist = distanceTo(local_map.detections[i].mapLocation.x, local_map.detections[i].mapLocation.y);
                                if (Ball_Dist < lowestDist)
                                {
                                    target = local_map.detections[i];
                                    lowestDist = Ball_Dist; 
                                    fprintf(fp,"\rFound Viable target at (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
                                }
                                else 
                                {
                                    fprintf(fp,"\rNo Viable target Found\n");
                                }
                            }
                        }
                    }
                }
            }
        }
    }  
    if(target.mapLocation.x < -3 || target.mapLocation.x > 3 )
        target.mapLocation.x = 0.00;

    if(target.mapLocation.y < -3 || target.mapLocation.y > 3 )
        target.mapLocation.y = 0.00;

    fprintf(fp,"\rReturning target at (%.2f, %.2f)\n",target.mapLocation.x, target.mapLocation.y);
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



bool getObject(bool CheckSide = true, bool CheckIso = false)
{
    bool HoldingBall = false; 
    int turn_step = 45;
    
    while(!HoldingBall)
    {    DETECTION_OBJECT target = findTarget(CheckSide,CheckIso);
        if(Balldetect.isNearObject() && CheckBallColor())
        {
            fprintf(fp,"\rThe robot is holding a Triball\n");
            Intake.stop(hold);
            HoldingBall = true;
        } 
       
        if(target.mapLocation.x && target.mapLocation.y != 0.00)
        {
            fprintf(fp,"\rFound Triball! || Triballl Location (%.2f, %.2f)\n", target.mapLocation.x, target.mapLocation.y);
            fprintf(fp,"\rProbability of this target being a Triball is %f%% \n", target.probability*100);
            Intake.spin(vex::directionType::fwd);
            moveToPosition(target.mapLocation.x * 100, target.mapLocation.y * 100,-1,true,75,75);
            vex::wait(250,msec);
        }
        else
        {
            //fprintf(fp,"\rSeanning for ball....\n");
            Chassis.turn_max_voltage = 3;
            fprintf(fp,"\rAngle to turn to %.2f Degrees\n",GPS.heading(deg) + turn_step);
            Chassis.turn_to_angle(GPS.heading(deg) + turn_step);
            vex::wait(1,sec);
            //target = findTarget(CheckSide, CheckIso);
        }
     
    }

    return HoldingBall;
}


void ScoreBall()
{   
    Point* Scoring_Point;
    double Scoring_Dir = 0;
    bool HoldingBall = true; 
    Intake.setVelocity(100,pct);
    if(field.In_Front_Score_Zone())
    {
        fprintf(fp,"\rIn Front Scoring Zone\n");
        Scoring_Point = field.Find_Scoring_Pos();// Find point on scoring line 
        fprintf(fp,"\rUnique point on line is (%.2f, %.2f)\n", Scoring_Point->Xcord, Scoring_Point->Ycord);

        if(field.Red_Side)
        {
            Scoring_Dir = 90.0;
        }
        else if(field.Blue_Side)
        {
            Scoring_Dir = 270.0;
        }   
    }
    else
    {
        double LeftSide_Dist = distanceTo(field.Score_Left.first->Xcord,field.Score_Left.first->Ycord,vex::distanceUnits::cm);
        double RightSide_Dist = distanceTo(field.Score_Right.first->Xcord,field.Score_Right.first->Ycord,vex::distanceUnits::cm);
        fprintf(fp, "\rRobot is not in Front Scoring Zone checking side point distances Left: %.2f || Right: %.2f\n", LeftSide_Dist, RightSide_Dist);

        fprintf (fp,"\rLeft Side Point(%.2f, %.2f) || Right Side Point(%.2f, %.2f)\n",field.Score_Left.first->Xcord, field.Score_Left.first->Ycord, field.Score_Right.first->Xcord, field.Score_Right.first->Ycord);
        if(LeftSide_Dist < RightSide_Dist)
        {
            Scoring_Point = field.Score_Left.first;
            Scoring_Dir = field.Score_Left.second;
        }
        else 
        {
            Scoring_Point = field.Score_Right.first;
            Scoring_Dir = field.Score_Right.second;
        }
    }
    
    fprintf(fp,"\rGoing to scoring point (%.2f, %.2f)\n",Scoring_Point->Xcord, Scoring_Point->Ycord);
    
    vex::wait(500,msec);

    while(HoldingBall)
    {
        moveToPosition(Scoring_Point->Xcord,Scoring_Point->Ycord,Scoring_Dir);
        Intake.spin(vex::directionType::rev);
        Chassis.drive_distance(20);
        if(!Balldetect.isNearObject())
        {
            fprintf(fp,"\rThe robot is not holding a Triball\n");
            Intake.stop(hold);
            HoldingBall = false;
        }
    }

    if(field.Red_Side)
        Chassis.turn_to_angle(270);
    else
        Chassis.turn_to_angle(90);

    //moveToPosition(-50,16.68125,-1,false);
}

#if defined(MANAGER_ROBOT)
bool GetMatchLoad()
{
    bool HoldingBall = false;
    double theta;
    if(field.Blue_Side)
    {
        theta = 45.00;
    }
    else if(field.Red_Side)
    {   
        theta = 225.0;
    }
    while(!HoldingBall)
    {

        if(Balldetect.isNearObject() && CheckBallColor())
        {
            fprintf(fp,"\rThe robot is holding a Triball\n");
            Intake.stop(hold);
            HoldingBall = true;
        } 
        moveToPosition(field.ML_Point->Xcord,field.ML_Point->Ycord,theta,false,100,100);
        Intake.spin(vex::directionType::fwd);
        Chassis.drive_distance(10);
        wait(500,msec);

    }
    
    return HoldingBall;



}
void Move2Drop_Pos()
{
bool HoldingBall = true; 
    double theta;
    if(field.Blue_Side)
        theta = 90.00; 
    else if(field.Red_Side)
        theta = 270.00;

    Point* DropPoint = field.Find_Drop_Pos();
    fprintf(fp,"\rTravelling to Drop Off Pos (%.2f, %.2f)\n",DropPoint->Xcord, DropPoint->Ycord);
    while(HoldingBall)
    {
        moveToPosition(DropPoint->Xcord,DropPoint->Ycord, 270,false,100,100);
        Chassis.drive_distance(15.00);
        Intake.spin(vex::directionType::rev);
        if(!Balldetect.isNearObject())
        {
            fprintf(fp,"\rThe robot is not holding a Triball\n");
            Intake.stop();
            HoldingBall = false;
        }
    }
}

void ThrowBall()
{
    if(CatapultLimit.pressing())
    {
        Catapult.spinFor(vex::directionType::fwd, 10 ,vex::rotationUnits::rev);
    }

    Catapult.spin(vex::directionType::fwd);
    while(!CatapultLimit.pressing())
    {
        wait(100,msec);
    }
    Catapult.stop(hold);
}

void BlockIntake()
{

}
#endif
