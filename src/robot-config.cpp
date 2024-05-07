#include "vex.h"
#include "robot-config.h"
using namespace std;

Field OverUnder ;

//Center Barrier Ref Points 
double 
TB_X_endpoints = 62.46,
TB_Y_endpoints = 119.69,
M_X_endpints = 0.00,
M_Y_endpints = 116.70;

const Point 
//Middle Line
M1(M_X_endpints,-M_Y_endpints),
M2(M_X_endpints, M_X_endpints),
//Top Line;
T1(-TB_X_endpoints,TB_Y_endpoints),
T2(TB_X_endpoints, TB_Y_endpoints),
//Bottom Line
B1(-TB_X_endpoints,-TB_Y_endpoints),
B2(TB_X_endpoints,-TB_Y_endpoints);

const Line 
ML(M1, M2),
TL(T1, T2),
BL(B1, B2);

const Barrier Center({BL,TL,ML});

//Goal Ref Points
double 
GC_X = 119.68,
G_Y = 59.85,
GS_X = 178.30;

//Red Goal
//Left Side Points
const Point 
FRC1(GC_X,G_Y), //Left - Front Red corner
RRC1(GS_X,G_Y), //Left - Rear Red corner
//Right Side Points
FRC2(GC_X,-G_Y), //Right - Front Red corner
RRC2(GS_X,-G_Y); //Right - Rear Red corner
//Goal Lines
const Line 
FR(FRC1,FRC2), // Front Red Line
LR(FRC1,RRC1), // Left Red Line 
RR(FRC2,RRC2); // Right Red Line
const Barrier Red_Goal({FR,LR,RR});

//Blue Goal
//Left Side Points
const Point 
FBC1(-GC_X,-G_Y), //Left - Front Blue corner
RBC1(-GS_X,-G_Y), //Left - Rear Blue corner
//Right Side Points
FBC2(-GC_X,G_Y), //Right - Front Blue corner
RBC2(-GS_X,G_Y); //Right - Rear Blue corner
//Goal Lines

const Line 
FB(FBC1,FBC2), // Front Blue Line
LB(FBC1,RBC1), // Left Blue Line 
RB(FBC2,RBC2); // Right Blue Line
const Barrier Blue_Goal({FB,LB,RB});

//ClearPath
double
CL1 = 95.00,
CL2 = 150.51,
CL_CP = 122.75,
GZ_XY = 61.35;

const Point
Q1_CP(CL1,CL2),
Q1_CCP(CL_CP,CL_CP),
Q1_GZP(GZ_XY,GZ_XY),
Q2_GZP(GZ_XY,-GZ_XY),
Q2_CCP(CL_CP,-CL_CP),
Q2_CP(CL1,-CL2),
Q3_CP(-CL1,-CL2),
Q3_CCP(-CL_CP,-CL_CP),
Q3_GZP(-GZ_XY,-GZ_XY),
Q4_GZP(-GZ_XY,GZ_XY),
Q4_CCP(-CL_CP,CL_CP),
Q4_CP(-CL1,CL2);
const Path FreePath({Q1_CP, Q1_CCP, Q1_GZP, Q2_GZP, Q2_CCP, Q2_CP, Q3_CP, Q3_CCP, Q3_GZP, Q4_GZP, Q4_CCP, Q4_CP});








void initialize_field()
{
    OverUnder.set_barriers({Center, Red_Goal, Blue_Goal});
    OverUnder.create_ClearPath(FreePath);
    

}

