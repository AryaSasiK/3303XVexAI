
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
    vector  <Line> BarrierLines;
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
    
    vector <Point *> PathPoints;
    Path() {}
    ~Path() {}
};

class Field
{
public:
    
    Field(vex::color Alliance_Color);
    vector<Point*> Path2Snap2;
    vector<Line*> P2S2_Lines;
    vector<const Barrier *> Field_Barriers;
    vector<const Point*> Goal_Zone;
    vex::color Side;
    pair<Point, double> Find_Closest_Point_In_Line(Point point, Line* LineSeg);
    pair<Point, int> Find_Point_on_Path(Point Target);
    bool Check_Barrier_Intersects(Point CurrentPos, Point PointOnLine);
    bool In_Goal_Zone(double Ball_x, double Ball_y);
    Path Create_Path_to_Target(Point Target);
    const Line* Find_Goal_Side();
    void Print_Lines();
};


