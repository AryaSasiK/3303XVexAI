
#pragma once
#include <vector>
#include "vex.h"
#include <robot-config.h>


using namespace std;


enum Color 
{
    Blue,
    Red,
    Purple
};

class Point 
{
    public:
    double Xcord, Ycord;
    Point(double X, double Y)
    {
        Xcord = X;
        Ycord = Y;
    }
    ~Point(){}
};

class Line 
{
    public:
        Point* LinePoints[2];
        Line(){}
        Line(Point A, Point B) : Line()
        {
            LinePoints[0] = &A;
            LinePoints[1] = &B;
        }
        ~Line()
        {

        }

};

class Barrier
{
    public:
        vector<Line*> BarrierLines;
        Barrier(std::initializer_list<Line> Barrier_List)
        {
            for (auto List : Barrier_List) 
            {
                BarrierLines.push_back(&List);
            }

        }
        ~Barrier(){}
};


class Path
{
    public: 
        std::vector<Point*> PathPoints;
        Path(){}
        Path(std::initializer_list<Point> PointsList) : Path()
        {
            for (auto List : PointsList) 
            {
                PathPoints.push_back(&List);
            }
           
        }
        ~Path(){}
       
};

class Field 
{
    private:
        const Path* Snap_Path ;
        std::vector<Line> Snap_Path_Lines;
        vector<Barrier*> Field_Barriers;
        
    public:
        Field(enum::Color Alliance);
        Color Side;
        pair <Point*, double> Find_Closest_Point_In_Line(Point* point, Line LineSeg);
        bool Check_Barrier_Intersects(Point* CurrentPos, Point* PointOnLine);
        pair<Point*,int> Find_Point_on_Path(Point* Target);
        Path Create_Path_to_Target(Point* Target);


};


