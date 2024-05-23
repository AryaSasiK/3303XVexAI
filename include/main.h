#include "vex.h"

void endgame();

void pidTuning();

void Position();

void printPosition (vex::distanceUnits units);

namespace matchload {
    void setCatapultDown();

    void catapultShoot();

    void startSubsystems();

    void ScoreAllianceTriball();

    void runMatchload (double time, vex::timeUnits  unit);

    void runMatchload (int loads);

    void ImproSwing(int LVel, int RVel, int Deg);

    void DemoTriball(int Preloads);
}