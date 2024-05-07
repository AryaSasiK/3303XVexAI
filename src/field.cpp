#include "field.h"


Point::~Point()
{
}

Line::~Line()
{
}

Barrier::~Barrier()
{
}

bool Path::findintersect(Point A1, Point A2, Point B1, Point B2)
{
    return false;
}

Path::~Path()
{
}

Field::Field()
{
    const double FC = 178.30;
    const Point C1(FC,FC), C2(FC,FC), C3(FC,FC), C4(FC,FC);
    const Line S1(C1,C2), S2(C2,C3), S3(C3,C4), S4(C4,C1);
    const Barrier Perimeter({S1,S2,S3,S4});
    Field_Barriers.push_back(&Perimeter);
}


