
#pragma once
#include <vector>
#include <robot-config.h>
#include "vex.h"
#include <algorithm>
#include <iostream>
#include <initializer_list> 
#include <cstdarg>

using namespace std;


class Point
{
public:
    double Xcord, Ycord;
    Point(){}
    Point(double X, double Y) //: Point()
    {
        Xcord = X;
        Ycord = Y;
    }
    ~Point() {}
};

class Line
{
public:
    pair<Point, Point> LinePoints;
    Line() {}
    Line(Point A, Point B)
    {
        LinePoints.first = A;
        LinePoints.second = B;
    }
    ~Line()
    {
    }
};

class Barrier
{
public:
    vector <Line> BarrierLines;
       Barrier(Line A , Line B, Line C )
    {
        BarrierLines.push_back(A);
        BarrierLines.push_back(B);
        BarrierLines.push_back(C);
    }

    ~Barrier() {}
};

class Path
{
public:
    double pathlength;
    vector <Point*> PathPoints;
    Path() {}
    ~Path() {}
    void calcPathLength();
};

class Field
{
    private:
    vector<Point*> Path2Snap2;
    vector<const Barrier *> Field_Barriers;
    vector<const Point*> Goal_Zone;
    vector<const Point*> ML_Zone;
    vector<const Point*> Isolation_Zone;
    vector<const Point*> Offensive_Zone;
    vector<const Point*> Scoring_Zone;
    vector<const Point*> Front_Scoring_Zone;
   
    
public:
    bool Red_Side;
    bool Blue_Side;
    double Width_Offset;
    double Front_Offset;
    pair<Point*,double> Score_Left;
    pair<Point*,double> Score_Right;
    Line* Score_Front;

    Field(bool isRed, double Robot_Width, double Intake_Offset);
    Point* Find_Scoring_Pos();
    Point* Find_Point_on_Path(Point* freePoint);
    bool Check_Barrier_Intersects(Point* point, Point* inPath, bool checkoffsets);
    bool In_Goal_Zone(float Ball_x, float Ball_y);
    bool In_MatchLoad_Zone(float Ball_x, float Ball_y);
    bool In_Iso_Zone(float Ball_x, float Ball_y, bool check);
    bool In_Offensive_Zone(float Ball_x, float Ball_y, bool check);
    bool In_Front_Score_Zone();
    pair<Point*, double> Dist_from_Node(int NodePos, Point* freePoint);
    Path Create_Path_to_Target(Point* Current, Point* Target);
    int getIndex(Point* AdjPoint);
    Line FindOffsetLines(Point*P1, Point* P2, bool offsettype);
    
};

