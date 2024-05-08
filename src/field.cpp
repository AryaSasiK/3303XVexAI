//#include "vex.h"
#include "field.h"
#include"OverUnder.h"


using namespace OverUnder;


Field::Field(enum::Color Alliance)
{
    Side = Alliance;
    Snap_Path = &Path2Snap2;
    Field_Barriers.push_back(&Center);
    Field_Barriers.push_back(&Blue_Goal);
    Field_Barriers.push_back(&Red_Goal);
}

bool Field::Check_Barrier_Intersects()
{
    return false;
}

// Point Field::Find_Point_on_Path()
// {
//     //return Point();
// }

// Path Field::Create_Path_to_Target()
// {

//     Point StartP = Find_Point_on_Path();
//     //return Path();
// }
