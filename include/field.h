
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
    Point(double X, double Y) : Point()
    {
        Xcord = X;
        Ycord = Y;
        

    }
    ~Point() {}
};

class Line
{
public:
    Point *LinePoints[2];
    Line() {}
    Line(Point A, Point B) : Line()
    {
        LinePoints[0] = &A;
        LinePoints[1] = &B;
    }
    Line(Point* A, Point* B) : Line()
    {
        LinePoints[0] = A;
        LinePoints[1] = B;
    }
    ~Line()
    {
    }
};

class Barrier
{
public:
    vector <Line *> BarrierLines;
    Barrier(Line A , Line B )
    {
        for (auto List : Barrier_List)
        {
            BarrierLines.push_back(&List);
        }
    }
    ~Barrier() {}
};

class Path
{
public:
    
    vector <Point *> PathPoints;
    Path() {}
    ~Path() {}
};

class Field
{
private:
    vector<const Barrier *> Field_Barriers;
    vector<const Point*> Goal_Zone;
    

public:
    
    Field(vex::color Alliance_Color);
    vector<const Point*> Path2Snap2;
    vector<const Line*> P2S2_Lines;
    vex::color Side;
    pair<Point *, double> Find_Closest_Point_In_Line(Point *point, const Line* LineSeg);
    pair<Point *, int> Find_Point_on_Path(Point *Target);
    bool Check_Barrier_Intersects(Point *CurrentPos, Point *PointOnLine);
    bool In_Goal_Zone(double Ball_x, double Ball_y);
    Path Create_Path_to_Target(Point *Target);
    const Line* Find_Goal_Side();
};


