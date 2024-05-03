
#pragma once
#include <vector>
enum Color 
{
    Blue,
    Red,
};


class Point 
{
    public:
    double Xcord,Ycord;
    Point(double X, double Y);
    ~Point();

      
};





class Path
{
    public:
    std::vector<Point> PlannedPath ;
    Path();
    ~Path();
};

class Barrier
{

};




class Field
{
    private:
  

    public:
    bool Red_Side; 
    bool Blue_Side; 
    Field(enum::Color Side);

     

    
};


























