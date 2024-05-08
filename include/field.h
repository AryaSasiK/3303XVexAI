
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
        ~Line(){}

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
        std::vector<Line*> PathLines;
        Path(std::initializer_list<Point> PointsList)
        {
            for (auto List : PointsList) 
            {
                PathPoints.push_back(&List);
            }
            for(int i = 0; i < (PathPoints.size() - 1); i++)
            {
                Line* newLine = new Line();
                newLine->LinePoints[0] = PathPoints[i];
                newLine->LinePoints[1] = PathPoints[i+1];
                PathLines.push_back(newLine);
            }
        }
        ~Path(){}
       
};

class Field 
{
    private:
        const Path* Snap_Path ;
        vector<Barrier*> Field_Barriers;
        
    public:
        Field(enum::Color Alliance);
        Color Side;
        bool Check_Barrier_Intersects();
        Point Find_Point_on_Path();
        Path Create_Path_to_Target();

};


