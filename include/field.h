
#pragma once
#include <vector>
using namespace std;

enum Color 
{
    Blue,
    Red,
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
    ~Point();
};

class Line 
{
    public:
        Point* LinePoints[2];
        Line(Point A, Point B) 
        {
            LinePoints[0] = &A;
            LinePoints[1] = &B;
        }
        ~Line();

};

class Barrier
{
    public:
        vector<const Line*> BarrierLines;
        Barrier(std::initializer_list<Line> Barrier_List)
        {
            for (auto& List : Barrier_List) 
            {
                BarrierLines.push_back(&List);
            }
        }
        ~Barrier();
};


class Path
{
    private:
        bool findintersect(Point A1, Point A2, Point B1, Point B2);
    public: 
        std::vector<const Point*> PathPoints;
         Path(std::initializer_list<Point> PointsList)
        {
             for (auto& List : PointsList) 
            {
                PathPoints.push_back(&List);
            }

        }
        ~Path();
       
};

class Field 
{
    private:
        vector<const Barrier*> Field_Barriers;
        const Path* Snap_Path ;
    public:
        Field();
        Color Side;
        void set_side(enum::Color Alliance_Side)
        {
            Side = Alliance_Side;
        }
        void set_barriers(std::initializer_list<Barrier> F_Barrs)
       {
            for (auto& List : F_Barrs) 
            {
                Field_Barriers.push_back(&List);
            }
        
       }
       void create_ClearPath(Path clearpath)
       {
            Snap_Path = &clearpath;
       }
    
        

};
