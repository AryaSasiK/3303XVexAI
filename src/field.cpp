#include "field.h"

static double
    Field_XY_Lim = 178.30,
    Alley_X = 62.46,
    Alley_Y = 119.69,
    CenterBar_X = 0.00,
    CenterBar_Y = 116.70;
static Point
    Q1_Field_Corner(Field_XY_Lim, Field_XY_Lim),
    Q2_Field_Corner(Field_XY_Lim, -Field_XY_Lim),
    Q3_Field_Corner(-Field_XY_Lim, -Field_XY_Lim),
    Q4_Field_Corner(-Field_XY_Lim, Field_XY_Lim);
static Line
    Front_Side_Field(Q4_Field_Corner, Q1_Field_Corner),
    Right_Side_Field(Q1_Field_Corner, Q2_Field_Corner),
    Rear_Side_Field(Q2_Field_Corner, Q3_Field_Corner),
    Left_Side_Field(Q3_Field_Corner, Q4_Field_Corner);

static Point
    Center_Bar1(CenterBar_X, -CenterBar_Y),
    Center_Bar2(CenterBar_X, CenterBar_Y),
    Blue_Alley1(-Alley_X, Alley_Y),
    Blue_Alley2(Alley_X, Alley_Y),
    Red_Alley1(-Alley_X, -Alley_Y),
    Red_Alley2(Alley_X, -Alley_Y);
static Line
    Center_Bar(Center_Bar1, Center_Bar2),
    Blue_Alley(Blue_Alley1, Blue_Alley2),
    Red_Alley(Red_Alley1, Red_Alley2);

// Goals Ref Cordinates(cm)
static double
    Goal_X = 119.68,
    Goal_Y = 59.85;

static Point
    Red_FL_Corner(Goal_X, Goal_Y),        // Front Left Corner
    Red_BL_Corner(Field_XY_Lim, Goal_Y),  // Back Left Corner
    Red_FR_Corner(Goal_X, -Goal_Y),       // Front Right Corner
    Red_BR_Corner(Field_XY_Lim, -Goal_Y); // Back Right Corner
static Line
    Red_Front_Side(Red_FL_Corner, Red_FR_Corner), // Front Red Line
    Red_Left_Side(Red_FL_Corner, Red_BL_Corner),  // Left Red Line
    Red_Right_Side(Red_FR_Corner, Red_BR_Corner); // Right Red Line

static Point
    Blue_FL_Corner(-Goal_X, -Goal_Y),       // Front Left Corner
    Blue_BL_Corner(-Field_XY_Lim, -Goal_Y), // Back Left Corner
    Blue_FR_Corner(-Goal_X, Goal_Y),        // Right - Front Blue corner
    Blue_BR_Corner(-Field_XY_Lim, Goal_Y);  // Right - Rear Blue corner
static Line
    Blue_Front_Side(Blue_FL_Corner, Blue_FR_Corner), // Front Blue Line
    Blue_Left_Side(Blue_FL_Corner, Blue_BL_Corner),  // Left Blue Line
    Blue_Right_Side(Blue_FR_Corner, Blue_BR_Corner); // Right Blue Line

// Path 2 Snap 2 Ref Cordinates
static double
    Match_Load_Ref1 = 95.00,
    Match_Load_Ref2 = 150.51,
    Match_Load_Center_XY = 122.75,
    Goal_Zone_XY = 61.35;

static Point
    Q1_Alley(Match_Load_Ref1, Match_Load_Ref2),
    Q1_Match_Load_Center(Match_Load_Center_XY, Match_Load_Center_XY),
    Q1_Goal_Zone(Goal_Zone_XY, Goal_Zone_XY),
    Q2_Goal_Zone(Goal_Zone_XY, -Goal_Zone_XY),
    Q2_Match_Load_Center(Match_Load_Center_XY, -Match_Load_Center_XY),
    Q2_Alley(Match_Load_Ref1, -Match_Load_Ref2),
    Q3_Alley(-Match_Load_Ref1, -Match_Load_Ref2),
    Q3_Match_Load_Center(-Match_Load_Center_XY, -Match_Load_Center_XY),
    Q3_Goal_Zone(-Goal_Zone_XY, -Goal_Zone_XY),
    Q4_Goal_Zone(-Goal_Zone_XY, Goal_Zone_XY),
    Q4_Match_Load_Center(-Match_Load_Center_XY, Match_Load_Center_XY),
    Q4_Alley(-Match_Load_Ref1, Match_Load_Ref2);

static Line
    Q1_LineA(Q1_Alley, Q1_Match_Load_Center),
    Q1_LineB(Q1_Match_Load_Center, Q1_Goal_Zone),
    Q1_LineC(Q1_Goal_Zone, Q2_Goal_Zone),
    Q2_LineA(Q2_Goal_Zone, Q2_Match_Load_Center),
    Q2_LineB(Q2_Match_Load_Center, Q2_Alley),
    Q2_LineC(Q2_Alley, Q3_Alley),
    Q3_LineA(Q3_Alley, Q3_Match_Load_Center),
    Q3_LineB(Q3_Match_Load_Center, Q3_Goal_Zone),
    Q3_LineC(Q3_Goal_Zone, Q4_Goal_Zone),
    Q4_LineA(Q4_Goal_Zone, Q4_Match_Load_Center),
    Q4_LineB(Q4_Match_Load_Center, Q4_Alley),
    Q4_LineC(Q4_Alley, Q1_Alley);

static Barrier
    RedGoal(Red_Front_Side, Red_Left_Side, Red_Right_Side),
    BlueGoal(Blue_Front_Side, Blue_Left_Side, Blue_Right_Side),
    CenterBarrier(Blue_Alley, Red_Alley, Center_Bar);
// const Barrier FieldPerimeter(Front_Side_Field,Right_Side_Field,Rear_Side_Field,Left_Side_Field);

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double distanceTo(Point *Point1, Point *Point2)
{
    double distance = sqrt(pow((Point2->Xcord - Point1->Xcord), 2) + pow((Point2->Ycord - Point1->Ycord), 2));
    return distance;
}

void Path::calcPathLength()
{
    for (int i = 0; i < PathPoints.size() - 1; i++)
    {
        pathlength = pathlength + distanceTo(PathPoints[i], PathPoints[i + 1]);
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Field::Field(vex::color Alliance_Color, double Robot_Offset)
{

    if (Alliance_Color == vex::color::red)
    {
        Red_Side = true;
        Blue_Side = false;
    }
    if (Alliance_Color == vex::color::blue)
    {
        Blue_Side = true;
        Red_Side = false;
    }

    Offset = Robot_Offset;

    Path2Snap2.push_back(&Q1_Alley);
    Path2Snap2.push_back(&Q1_Match_Load_Center);
    Path2Snap2.push_back(&Q1_Goal_Zone);
    Path2Snap2.push_back(&Q2_Goal_Zone);
    Path2Snap2.push_back(&Q2_Match_Load_Center);
    Path2Snap2.push_back(&Q2_Alley);
    Path2Snap2.push_back(&Q3_Alley);
    Path2Snap2.push_back(&Q3_Match_Load_Center);
    Path2Snap2.push_back(&Q3_Goal_Zone);
    Path2Snap2.push_back(&Q4_Goal_Zone);
    Path2Snap2.push_back(&Q4_Match_Load_Center);
    Path2Snap2.push_back(&Q4_Alley);

    P2S2_Lines.push_back(&Q1_LineA);
    P2S2_Lines.push_back(&Q1_LineB);
    P2S2_Lines.push_back(&Q1_LineC);
    P2S2_Lines.push_back(&Q2_LineA);
    P2S2_Lines.push_back(&Q2_LineB);
    P2S2_Lines.push_back(&Q2_LineC);
    P2S2_Lines.push_back(&Q3_LineA);
    P2S2_Lines.push_back(&Q3_LineB);
    P2S2_Lines.push_back(&Q3_LineC);
    P2S2_Lines.push_back(&Q4_LineA);
    P2S2_Lines.push_back(&Q4_LineB);
    P2S2_Lines.push_back(&Q4_LineC);

    Field_Barriers.push_back(&CenterBarrier);
    Field_Barriers.push_back(&BlueGoal);
    Field_Barriers.push_back(&RedGoal);

    Goal_Zone.push_back(&Red_BL_Corner);
    Goal_Zone.push_back(&Red_FL_Corner);
    Goal_Zone.push_back(&Red_FR_Corner);
    Goal_Zone.push_back(&Red_BR_Corner);
}

pair<Point, double> Field::Find_Closest_Point_In_Line(Point point, Line *LineSeg)
{

    pair<Point, double> Data;
    double
        Ax = LineSeg->LinePoints.first.Xcord,
        Ay = LineSeg->LinePoints.first.Ycord,
        Bx = LineSeg->LinePoints.second.Xcord,
        By = LineSeg->LinePoints.second.Ycord,
        Pointx = point.Xcord,
        Pointy = point.Ycord;
    // fprintf(fp,"Line Seg Point A (%.2f, %.2f), Point B (%.2f, %.2f), Point C(%.2f, %.2f)",Ax,Ay,Bx,By,Pointx,Pointy);
    double Px = Bx - Ax;
    double Py = By - Ay;
    double temp = (Px * Px) + (Py * Py);
    double U = ((Pointx - Ax) * Px + (Pointy - Ay) * Py) / (temp);
    if (U > 1)
    {
        U = 1;
    }
    else if (U < 0)
    {
        U = 0;
    }
    double X = Ax + U * Px;
    double Y = Ay + U * Py;
    double Dx = X - Pointx;
    double Dy = Y - Pointy;
    double Dist = sqrt((Dx * Dx) + (Dy * Dy));

    // Point* Point_On_Line = new Point(X,Y);
    Data.first = Point(X, Y);
    Data.second = Dist;

    // fprintf(fp,"Point on Line:(%.2f, %.2f) Distance: %.2f\n",X,Y,Dist);

    return Data;
}

int orientation(Point p, Point q, Point r)
{
    double val = (q.Ycord - p.Ycord) * (r.Xcord - q.Xcord) - (q.Xcord - p.Xcord) * (r.Ycord - q.Ycord);
    if (val == 0)
        return 0; // collinear

    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool Check_Intersects(Point CurrentPos, Point PointOnLine, Line BarrierLine)
{
    Point Line_PointA = BarrierLine.LinePoints.first;
    Point Line_PointB = BarrierLine.LinePoints.second;
    int o1 = orientation(CurrentPos, PointOnLine, Line_PointA);
    int o2 = orientation(CurrentPos, PointOnLine, Line_PointB);
    int o3 = orientation(Line_PointA, Line_PointB, CurrentPos);
    int o4 = orientation(Line_PointA, Line_PointB, PointOnLine);
    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    return false; // Doesn't fall in any of the above cases
}

Line Field::FindOffsetLines(Point P1, Point P2, bool offsettype)
{


    double dx = P2.Xcord - P1.Xcord;
    double dy = P2.Ycord - P1.Ycord;
    double Length = sqrt(pow(dx,2) + pow(dy,2));
    double Dx = dy * Offset/Length;
    double Dy = -dx * Offset/Length;
    Point ModP1;
    Point ModP2;

    if(offsettype)
    {
        ModP1 = Point(P1.Xcord+Dx,P1.Ycord+Dy);
        ModP2 = Point(P2.Xcord+Dx,P2.Ycord+Dy);
    }
    else
    {
        ModP1 = Point(P1.Xcord-Dx,P1.Ycord-Dy);
        ModP2 = Point(P2.Xcord-Dx,P2.Ycord-Dy);
    }

    Line ParallelLine(ModP1,ModP2);

    return ParallelLine;
}


bool Field::Check_Barrier_Intersects(Point CPos, Point POL, bool checkoffsets = false)
{   
    Line LineA = FindOffsetLines(CPos,POL,true);
    Line LineB = FindOffsetLines(CPos,POL,false);
        // fprintf(fp,"Current Pos (%.2f, %.2f), Point on Line(%.2f, %.2f)",CPos.Xcord,CPos.Ycord,POL.Xcord,POL.Ycord);
    bool Intersect = false;
    for (int i = 0; i < Field_Barriers.size(); i++)
    {
        for (int j = 0; j < Field_Barriers[i]->BarrierLines.size(); j++)
        {
            
            if (Check_Intersects(CPos, POL, Field_Barriers[i]->BarrierLines[j]))
            {
                return true;
            }
            if(checkoffsets)
            {
                if(Check_Intersects(LineA.LinePoints.first, LineA.LinePoints.second, Field_Barriers[i]->BarrierLines[j]))
                {
                    return true;
                }
                if(Check_Intersects(LineB.LinePoints.first, LineB.LinePoints.second, Field_Barriers[i]->BarrierLines[j]))
                {
                    return true;
                }
            }
        }
    }
    return Intersect;
}

bool Field::In_Goal_Zone(float Ball_x, float Ball_y)
{
    Ball_x = Ball_x * 100;
    Ball_y = Ball_y * 100;
    int num_vertices = Goal_Zone.size();
    float x = fabs(Ball_x), y = Ball_y;
    bool inside = false;
    Point P1(Goal_Zone[0]->Xcord, Goal_Zone[0]->Ycord);
    Point P2;
    // Loop through each edge in the polygon
    for (int i = 1; i <= num_vertices; i++)
    {
        P2 = Point(Goal_Zone[i % num_vertices]->Xcord, Goal_Zone[i % num_vertices]->Ycord);
        if (y > min(P1.Ycord, P2.Ycord))
        {
            if (y <= max(P1.Ycord, P2.Ycord))
            {
                if (x <= max(P1.Xcord, P2.Xcord))
                {
                    double x_intersection = (y - P1.Ycord) * (P2.Xcord - P1.Xcord) / (P2.Ycord - P1.Ycord) + P1.Xcord;
                    if (P1.Xcord == P2.Xcord || x <= x_intersection)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        P1 = P2;
    }
    return inside;
}

bool pairCompare(const std::pair<Point, double> &firstElem, const std::pair<Point, double> &secondElem)
{
    return firstElem.second < secondElem.second;
}

pair<Point, int> Field::Find_Point_on_Path(Point freePoint)
{
    vector<pair<Point, double>> Point_Dist; // Vector that stores a Point which lies on a line in our Path2Snap2 and the distance from target
    vector<double> PiL;                     // This serves as a copy of the distances
    pair<Point, int> Point_PiP;             // This holds the closest point to a line on the Snap2path and its line position on that path
    pair<Point, double> temp;

    for (int i = 0; i < P2S2_Lines.size(); i++)
    {
        temp = Find_Closest_Point_In_Line(freePoint, P2S2_Lines[i]);
        Point_Dist.push_back(temp); //
        PiL.push_back(temp.second); // This will hold the distance in the order of the path lines
    }

    // if(Point_Dist.size() > 1 && PiL.size() > 1)
    //     fprintf(fp,"Point_Dist & Line_Pos vectors are populated");
    // else
    //     fprintf(fp,"Point_Dist & Line_Pos vectors are empty");

    sort(Point_Dist.begin(), Point_Dist.end(), pairCompare);

    // for(int k = 0; k < Point_Dist.size(); k++)
    // {
    //     fprintf(fp,"Sorted Vector Point %i is at coordinate (%.2f, %.2f) with a distance %.2f \n", k, Point_Dist[k].first.Xcord, Point_Dist[k].first.Ycord, Point_Dist[k].second);
    // }

    for (int j = 0; j < Point_Dist.size(); j++)
    {
        if (!Check_Barrier_Intersects(freePoint, Point_Dist[j].first))
        {

            Point_PiP.first.Xcord = Point_Dist[j].first.Xcord;
            Point_PiP.first.Ycord = Point_Dist[j].first.Ycord;
            for (int k = 0; k < PiL.size(); k++)
            {
                if (PiL[k] == Point_Dist[j].second)
                {
                    Point_PiP.second = k;
                    return Point_PiP;
                }
            }
        }
    }
    return Point_PiP;
}

Path Field::Create_Path_to_Target(Point Target)
{
    Path DrivePath;
    Point CurrentPos(GPS.xPosition(vex::distanceUnits::cm), GPS.yPosition(vex::distanceUnits::cm));
    // Point CurrentPos(-40, -40);
    Point *temp;
    pair<Point, int> Start = Find_Point_on_Path(CurrentPos);
    pair<Point, int> End = Find_Point_on_Path(Target);
    vector<int> PathA;
    vector<int> PathB;
    Path pathA;
    Path pathB;
    int StartingLine = Start.second + 1;
    int EndingLine = End.second + 1;

    for (int i = StartingLine; i != EndingLine; i--)
    {
        PathA.push_back(i - 1);
        if (i == 1)
        {
            i = Path2Snap2.size() + 1;
        }
    }
    for (int i = StartingLine; i != EndingLine; i++)
    {
        if (i == Path2Snap2.size())
        {
            i = 0;
        }
        PathB.push_back(i);
    }
    int pos;
    // fprintf(fp, "Current Pos: (%.2f, %.2f) - > Target Pos: (%.2f, %.2f)\n", CurrentPos.Xcord, CurrentPos.Ycord, Target.Xcord, Target.Ycord);
    // fprintf(fp, "First point to drive to is (%.2f, %.2f)\n", Start.first.Xcord, Start.first.Ycord);
    
    pathA.PathPoints.push_back(&CurrentPos);
    pathA.PathPoints.push_back(&Start.first);
    for (int i = 0; i < PathA.size(); i++)
    {
        pos = PathA[i];
        temp = new Point(Path2Snap2[pos]->Xcord, Path2Snap2[pos]->Ycord);
        pathA.PathPoints.push_back(temp);
        //fprintf(fp, " -> (%.2f, %.2f)", temp->Xcord, temp->Ycord);
    }
    pathA.PathPoints.push_back(&End.first);
    pathA.PathPoints.push_back(&Target);
    pathA.calcPathLength();

    pathB.PathPoints.push_back(&CurrentPos);
    pathB.PathPoints.push_back(&Start.first);
    for (int i = 0; i < PathB.size(); i++)
    {
        pos = PathB[i];
        temp = new Point(Path2Snap2[pos]->Xcord, Path2Snap2[pos]->Ycord);
        pathB.PathPoints.push_back(temp);
        //fprintf(fp, " -> (%.2f, %.2f)", temp->Xcord, temp->Ycord);
    }
    pathB.PathPoints.push_back(&End.first);
    pathB.PathPoints.push_back(&Target);
    pathB.calcPathLength();

    if(pathA.pathlength < pathB.pathlength)
    {
        for(int i = 0; i < pathA.PathPoints.size(); i++)
        {
            DrivePath.PathPoints.push_back(pathA.PathPoints[i]);
            // fprintf(fp, " -> (%.2f, %.2f)", pathA.PathPoints[i]->Xcord, pathA.PathPoints[i]->Ycord);
        }
    }
    else
    {
         for(int i = 0; i < pathB.PathPoints.size(); i++)
        {
            DrivePath.PathPoints.push_back(pathB.PathPoints[i]);
            // fprintf(fp, " -> (%.2f, %.2f)", pathB.PathPoints[i]->Xcord, pathB.PathPoints[i]->Ycord);
        }
    }

    // fprintf(fp, "- > (%.2f, %.2f) Last point before target ", End.first.Xcord, End.first.Ycord);
    // fprintf(fp, "|| Target point is (%.2f, %.2f)\n", Target.Xcord, Target.Ycord);

    return DrivePath;
}


