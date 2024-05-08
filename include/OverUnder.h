#include "field.h"

namespace OverUnder
{

//Center Barrier Ref Cordinates
double 
Field_XY_Lim = 178.30,
Alley_X = 62.46,
Alley_Y = 119.69,
CenterBar_X = 0.00,
CenterBar_Y = 116.70;

Point 
Q1_Field_Corner(Field_XY_Lim,Field_XY_Lim),
Q2_Field_Corner(Field_XY_Lim,-Field_XY_Lim),
Q3_Field_Corner(-Field_XY_Lim,-Field_XY_Lim),
Q4_Field_Corner(-Field_XY_Lim,Field_XY_Lim);
Line
Front_Side_Field(Q4_Field_Corner,Q1_Field_Corner),
Right_Side_Field(Q1_Field_Corner,Q2_Field_Corner),
Rear_Side_Field(Q2_Field_Corner,Q3_Field_Corner),
Left_Side_Field(Q3_Field_Corner,Q4_Field_Corner);
Barrier Perimeter({Front_Side_Field,Right_Side_Field,Rear_Side_Field,Left_Side_Field});

Point 
Center_Bar1(CenterBar_X,-CenterBar_Y),
Center_Bar2(CenterBar_X, CenterBar_Y),
Blue_Alley1(-Alley_X,Alley_Y),
Blue_Alley2(Alley_X, Alley_Y),
Red_Alley1(-Alley_X,-Alley_Y),
Red_Alley2(Alley_X,-Alley_Y);
Line 
Center_Bar(Center_Bar1, Center_Bar2),
Blue_Alley(Blue_Alley1, Blue_Alley2),
Red_Alley(Red_Alley1, Red_Alley2);
Barrier Center({Blue_Alley,Red_Alley,Center_Bar});

//Goals Ref Cordinates(cm)
double 
Goal_X = 119.68,
Goal_Y = 59.85;
Point 
Red_FL_Corner(Goal_X,Goal_Y), //Front Left Corner 
Red_BL_Corner(Field_XY_Lim ,Goal_Y),//Back Left Corner
Red_FR_Corner(Goal_X,-Goal_Y),//Front Right Corner 
Red_BR_Corner(Field_XY_Lim ,-Goal_Y);//Back Right Corner
Line 
Red_Front_Side(Red_FL_Corner,Red_FR_Corner), // Front Red Line
Red_Left_Side(Red_FL_Corner,Red_BL_Corner), // Left Red Line 
Red_Right_Side(Red_FR_Corner,Red_BR_Corner); // Right Red Line
Barrier Red_Goal({Red_Front_Side,Red_Left_Side,Red_Right_Side});

Point 
Blue_FL_Corner(-Goal_X,-Goal_Y), //Front Left Corner
Blue_BL_Corner(-Field_XY_Lim ,-Goal_Y), //Back Left Corner
Blue_FR_Corner(-Goal_X,Goal_Y), //Right - Front Blue corner
Blue_BR_Corner(-Field_XY_Lim ,Goal_Y); //Right - Rear Blue corner
Line 
Blue_Front_Side(Blue_FL_Corner,Blue_FR_Corner), // Front Blue Line
Blue_Left_Side(Blue_FL_Corner,Blue_BL_Corner), // Left Blue Line 
Blue_Right_Side(Blue_FR_Corner,Blue_BR_Corner); // Right Blue Line
Barrier Blue_Goal({Blue_Front_Side,Blue_Left_Side,Blue_Right_Side});

// Path 2 Snap 2 Ref Cordinates
double
Match_Load_Ref1 = 95.00,
Match_Load_Ref2 = 150.51,
Match_Load_Center_XY = 122.75,
Goal_Zone_XY = 61.35;

Point
Q1_Alley(Match_Load_Ref1,Match_Load_Ref2),
Q1_Match_Load_Center(Match_Load_Center_XY,Match_Load_Center_XY),
Q1_Goal_Zone(Goal_Zone_XY,Goal_Zone_XY),
Q2_Goal_Zone(Goal_Zone_XY,-Goal_Zone_XY),
Q2_Match_Load_Center(Match_Load_Center_XY,-Match_Load_Center_XY),
Q2_Alley(Match_Load_Ref1,-Match_Load_Ref2),
Q3_Alley(-Match_Load_Ref1,-Match_Load_Ref2),
Q3_Match_Load_Center(-Match_Load_Center_XY,-Match_Load_Center_XY),
Q3_Goal_Zone(-Goal_Zone_XY,-Goal_Zone_XY),
Q4_Goal_Zone(-Goal_Zone_XY,Goal_Zone_XY),
Q4_Match_Load_Center(-Match_Load_Center_XY,Match_Load_Center_XY),
Q4_Alley(-Match_Load_Ref1,Match_Load_Ref2);

Path Path2Snap2 ({Q1_Alley, Q1_Match_Load_Center, Q1_Match_Load_Center, Q2_Match_Load_Center, Q2_Match_Load_Center, Q2_Alley, Q3_Alley, Q3_Match_Load_Center, Q3_Match_Load_Center, Q4_Match_Load_Center, Q4_Match_Load_Center, Q4_Alley, Q1_Alley});
}