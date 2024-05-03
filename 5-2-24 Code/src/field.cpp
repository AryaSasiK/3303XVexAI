#include "field.h"

Point::Point(double X, double Y)
{
    this->Xcord = X;
    this->Ycord = Y;
}

Point::~Point()
{
}

Path::Path()
{

}

Path::~Path()
{

}

Field::Field(enum::Color Side)
{
    if (Side == Blue)
    {
        Blue_Side = true;
        Red_Side = false;
    }

    if (Side == Red)
    {
        Red_Side = true;
        Blue_Side =false;
    }
}


