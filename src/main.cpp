/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       james                                                     */
/*    Created:      Mon Aug 31 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "ai_functions.h"
#include "main.h"

using namespace std;
using namespace vex;

//Realsense Offsets (15in, 24in)
// X(0.25in,0in), Y(-4.25in,12in), Z(9.125in,11in), Heading(0,0), Elevation(0,0)

// GPS Offsets (15in, 24in)
// X(0,0), Y(-6.5in,8in), Z(9.875,11in), Heading(180,180)



// File for fprintf function
FILE *fp = fopen("/dev/serial2","wb");
// Standard objects
brain Brain;
controller Controller1 = controller(primary);
competition Competition;
ai::jetson  jetson_comms;// create instance of jetson class to receive location and other

#define  MANAGER_ROBOT    1

#if defined(MANAGER_ROBOT)
#pragma message("building for the manager")
ai::robot_link       link( PORT21, "24in 3303X", linkType::manager );
//24in Robot Specific Objects
motor leftDriveA = motor(PORT9, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT7, ratio6_1, true);   
motor leftDriveD = motor(PORT8, ratio6_1, true);
motor rightDriveA = motor(PORT3, ratio6_1, false);
motor rightDriveB = motor(PORT4, ratio6_1, false);
motor rightDriveC = motor(PORT1, ratio6_1, false);
motor rightDriveD = motor(PORT2, ratio6_1, false);
motor Catapult = motor(PORT11,ratio36_1,true);
optical Balldetect = optical(PORT6);
gps GPS = gps(PORT20, 0.0, -163, mm, 180);
const int32_t InertialPort = PORT18;
const int32_t HangAPort = PORT14;
const int32_t HangBPort = PORT13;
const int32_t IntakePort = PORT12;
rotation hangEncoder = rotation(PORT5);
rotation catapultEncoder = rotation(PORT16);
limit catapultLimit = limit(Brain.ThreeWirePort.D);
digital_out rightWings = digital_out(Brain.ThreeWirePort.C);
digital_out leftWings = digital_out(Brain.ThreeWirePort.E);
digital_out ratchet = digital_out(Brain.ThreeWirePort.H);
double Robot_x_Offset = 9.5;
double Intake_Offset = 11;

#else
#pragma message("building for the worker")
ai::robot_link       link( PORT21, "15in 3303X", linkType::worker );
//15in Robot Specific Objects
motor leftDriveA = motor(PORT20, ratio6_1, true);  
motor leftDriveB = motor(PORT10, ratio6_1, true);   
motor leftDriveC = motor(PORT19, ratio6_1, false);   
motor leftDriveD = motor(PORT9, ratio6_1, false);   
motor rightDriveA = motor(PORT12, ratio6_1, false);
motor rightDriveB = motor(PORT2, ratio6_1, false);
motor rightDriveC = motor(PORT7, ratio6_1, true);
motor rightDriveD = motor(PORT4, ratio6_1, true);
const int32_t IntakePort = PORT11;
gps GPS = gps(PORT3, 0.0, -146, mm, 180);
const int32_t InertialPort = PORT16;
const int32_t HangAPort = PORT15;
const int32_t HangBPort = PORT13;
double Robot_x_Offset = 1;

#endif
#define AtanFunction abs(63*atan(0.07*(78-catapultEncoder.angle(degrees))))
#define IsoTimeEnd 84
#define InterTimeEnd 140
bool isoStart = false;
float generalTimer = 0 ;
  

// Field object for path following
Field field(true,Robot_x_Offset,Intake_Offset);
//FILE *fp = fopen("/dev/serial2","wb");
// Universal Objects (Do not comment out)
motor_group LeftDriveSmart = motor_group(leftDriveA, leftDriveB, leftDriveC, leftDriveD);
motor_group RightDriveSmart = motor_group(rightDriveA, rightDriveB, rightDriveC,rightDriveD);
Drive Chassis(LeftDriveSmart,RightDriveSmart,InertialPort, 3.125, 0.6, 360);
motor HangA = motor(HangAPort, ratio36_1, false);
motor HangB = motor(HangBPort, ratio36_1, true);
motor_group Hang = motor_group(HangA, HangB);
motor Intake = motor(IntakePort, ratio6_1, true);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void tuned_constants()
{
  Chassis.set_drive_constants(12, 1.5, 0, 10, 0);//Not tuned, but working
  Chassis.set_heading_constants(6, .4, 0, 1, 0);
  Chassis.set_turn_constants(12, 0.25, 0.015, 1.1, 15);//Tuned
  Chassis.set_swing_constants(12, .25, .015, 1.1, 15);//Tuned
  Chassis.set_drive_exit_conditions(1.5, 300, 2000);
  Chassis.set_turn_exit_conditions(1, 300, 2000);
  Chassis.set_swing_exit_conditions(1, 300, 1000);
}

void pre_auton(void) 
{
  tuned_constants();
  Brain.Screen.clearScreen();
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  Chassis.Gyro.calibrate();
  Brain.Screen.print("Calibrating Inertial for Chassis");
  Brain.Screen.setCursor(3, 1);
  // wait for the Inertial calibration process to finish
  while (Chassis.Gyro.isCalibrating()) 
  {
    wait(25, msec);
  }
  GPS.calibrate();
  while (GPS.isCalibrating()) 
  {
    wait(25, msec);
  }
  wait(50, msec);
  Brain.Screen.clearScreen();
}

bool Controller1LeftShoulderControlMotorsStopped = true;
void usercontrol(void) 
{
  Intake.setVelocity(100,pct);
  while(1)
  {
    Chassis.control_arcade();

    if (Controller1.ButtonL1.pressing()) 
    {
      Intake.spin(vex::directionType::fwd);
      Controller1LeftShoulderControlMotorsStopped = false;
    } 
    else if (Controller1.ButtonL2.pressing()) 
    {
      Intake.spin(vex::directionType::rev);
      Controller1LeftShoulderControlMotorsStopped = false;
    } 
    else if (!Controller1LeftShoulderControlMotorsStopped) 
    {
      Intake.stop();
      Controller1LeftShoulderControlMotorsStopped = true;
    }
    wait(20,msec);
  }
}

void testing_tuning(void)
{
   while(true)
  {
    if(getObject(1,1))
    {
      ScoreBall();
      wait(2, sec);
    }
  }
}


void pidTuning()
{
  Chassis.turn_to_angle(90);
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          Auto_Isolation Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous isolation  */
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Isolation(void) 
{
 
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                        Auto_Interaction Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous interaction*/
/*  phase of a VEX AI Competition.                                           */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void auto_Interaction(void) 
{

  //getObject(true,false);
  isoStart = true;
//matchload::startSubsystems(false);
//matchload::ScoreAllianceTriball();
//matchload::runMatchload(10);
  
  matchload::Hanging();

fprintf(fp,"\rTotal time: %.1f seconds\n" , ((generalTimer)*0.53));

  //matchload::runMatchload(3);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                          AutonomousMain Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*---------------------------------------------------------------------------*/

bool firstAutoFlag = true;
void autonomousMain(void) 
{
  // ..........................................................................
  // The first time we enter this function we will launch our Isolation routine
  // When the field goes disabled after the isolation period this task will die
  // When the field goes enabled for the second time this task will start again
  // and we will enter the interaction period. 
  // ..........................................................................

  if(firstAutoFlag)
    auto_Isolation();
  else 
    auto_Interaction();

  firstAutoFlag = false;
}

/**
 * Contains the tasks need to be accomplished at the end of the game (hang, etc.)
*/
void endgame () {
  
  Point goalAlignPos = Point(65.00, 0);
  moveToPoint(&goalAlignPos);
  
}

int main() {

  // local storage for latest data from the Jetson Nano
  static AI_RECORD       local_map;
  // Run at about 15Hz
  int32_t loop_time = 33;
  // start the status update display
  thread t1(dashboardTask);
  pre_auton(); 
  // Set up callbacks for autonomous and driver control periods.
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(auto_Interaction);
  // Competition.autonomous(autonomousMain);
  this_thread::sleep_for(loop_time);
  int counter = 0 ;
  
  while(1) 
  {

      jetson_comms.get_data( &local_map ); // get last map data
      link.set_remote_location( local_map.pos.x, local_map.pos.y, local_map.pos.az, local_map.pos.status );// set our location to be sent to partner robot
      counter += 1 ;


      if (counter > 15)
      {
        //printPosition(distanceUnits::cm);
        //testing_tuning();  
        fprintf(fp,"\rPositional Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",local_map.pos.az,local_map.pos.x*100,local_map.pos.y*100);
        fprintf(fp,"\rGPS Positional Data || Azimuth:%.2f Degrees X:%.2f cm Y:%.2f cm\n",GPS.heading(vex::rotationUnits::deg), GPS.xPosition(vex::distanceUnits::cm),GPS.yPosition(vex::distanceUnits::cm));
        //fprintf(fp,"\rTimer is: %.1f\n" , generalTimer);
        
        if(isoStart == true)
          generalTimer += 1 ;

        counter = 0 ;
      }
      // request new data    
      // NOTE: This request should only happen in a single task.    
      jetson_comms.request_map();
      // Allow other tasks to run
      this_thread::sleep_for(loop_time);
  }
}


void printPosition (vex::distanceUnits units = vex::distanceUnits::cm)
{
  fprintf(fp,"Actual position is: X: %.2f ", GPS.xPosition(units));
  fprintf(fp, "and Y: %.2f \n", GPS.yPosition(units));  
}


/**
 * Catapult control functions
*/

namespace matchload {

  #define HangPos 125
  float HangTurns = 0;
  bool LimitControl = false;

  /**
   * This sets the catapult in the down position
  */
  void setCatapultDown (bool Shoot = true)
  {

    if(Shoot == true)
    {
    while (LimitControl == false)
      {
        if(catapultEncoder.angle() > 250)
        Catapult.spin(fwd,-70,percent);

        else
        Catapult.spin(fwd,-(AtanFunction),percent);
        
        if(catapultLimit.pressing() || (catapultEncoder.angle(degrees) > 70 && catapultEncoder.angle(degrees) < 150))
        LimitControl = true;

      }
      Catapult.stop();
    }

    else
    {
      while(catapultEncoder.angle(degrees) < 50 || catapultEncoder.angle(degrees) > 65)
        {

          if(catapultEncoder.angle(degrees) < 80)
        {
            Catapult.spin(fwd,-(AtanFunction),percent);
        }  
          else
            Catapult.spin(fwd,-30,percent);
        
        }
        
        Catapult.stop();
    }

  }


void checkPosition(float Degs)
{
  HangTurns = ((Degs-(hangEncoder.angle(degrees)))*14);
  wait(25,msec);
}


  void catapultShoot ()
  {

    Catapult.spinFor(fwd,-250,degrees);
    LimitControl = false;
    setCatapultDown();
  }

  /**
   * Starts the various subsystems at te start of the game.
   * Sets the hang in the down position, releases intake, and TargetLoads catapult.
  */
  void startSubsystems(bool isShooting = true)
  {
  //  Hang.spin(fwd);
  //  waitUntil(hangEncoder.angle(degrees) >= 50);
    Intake.setVelocity(100, pct);
    Intake.spin(fwd);
    //Catapult.spinFor(fwd,-100,degrees);
    if(isShooting == true)
    setCatapultDown();

    else
    setCatapultDown(false);

    Catapult.stop();
   // waitUntil(hangEncoder.angle(degrees) >= 200);
    Hang.stop();
    Intake.stop();
    wait(600,msec);
  }


  void runMatchload (double time, vex::timeUnits unit) {

  }


  void ScoreAllianceTriball()
  {
    checkPosition(230);
    Hang.spinFor(fwd,HangTurns,degrees,false);
    Intake.spin(fwd,-100,percent);
    LeftDriveSmart.spin(fwd,50,percent);
    RightDriveSmart.spin(fwd,50,percent);
    wait(300,msec);
    Balldetect.objectDetectThreshold(30);
    int timeout2 = 0;
    bool loseball = false;
    while(!Balldetect.isNearObject())
    {
      timeout2 += 1;
      wait(25,msec);
      Intake.spin(fwd,-100,percent);
      if (timeout2 > 200)
       loseball = true;

      if(loseball == true)
      {
        Chassis.drive_distance(-5);
        Chassis.drive_distance(5);
        timeout2 = 0;
        loseball = false;
      }

    }
    Intake.stop();
    //moveToPosition(80,85,270,false,75,80);
    //


//Chassis.drive_distance(-3);
Chassis.right_swing_to_angle(90);
Chassis.left_swing_to_angle(175);
//moveToPosition(140,120,170,true,100,100);
Intake.spin(fwd);
wait(700,msec);
Chassis.drive_distance(10);
Chassis.drive_distance(-10);
Chassis.drive_distance(10);
ImproSwing(-100,-20,1500);
moveToPosition(120,120,45,false,100,100);

/*
    
  ImproSwing(-80,-100,2000);
//  ImproSwing(-100,-50,500);
  moveToPosition(45, 45, 90,false,100,100);
  Chassis.turn_to_angle(90);
  Chassis.drive_distance(10);
  Intake.spin(fwd,100,pct);
  wait(500,msec);
  Chassis.drive_distance(12);
  Chassis.drive_distance(-6);
  Chassis.drive_distance(5);
  Chassis.drive_distance(-7);
  Intake.stop();
  Chassis.turn_to_angle(0);
  ImproSwing(100,60,1700);
  moveToPosition(120,120,45,false,100,100);
  Intake.spin(fwd,-100,pct);
*/

  }

  void runMatchload (int TargetLoads) {
    
    setCatapultDown();
    Intake.spin(fwd,-100,pct);
    bool Stuck = false;
    float Loads = 0;
    LeftDriveSmart.spin(fwd,40,percent);
    RightDriveSmart.spin(fwd,40,percent);
    wait(400,msec);

      while ( Loads < TargetLoads)
      {

        int timeout = 0;
        Balldetect.objectDetectThreshold(30);
        //waitUntil(Balldetect.brightness() > 70 && Balldetect.brightness() < 110);
        //while(!(Balldetect.hue() > BlueMinHue && Balldetect. hue() < BlueMaxHue))
        while(!Balldetect.isNearObject())
        {
          Intake.spin(fwd,-100,percent);
          timeout += 1;
          wait(25,msec);
          if(timeout > 200)
          {
            Stuck = true;
            break;
          }
        }
        if(Stuck == true)
          {
            Intake.spin(fwd);
            Chassis.drive_distance(-15);
            Intake.spin(fwd,-100,pct);
            findTarget(true,false);
            getObject(true,false);
            Stuck = false;
          }

        LeftDriveSmart.stop();
        RightDriveSmart.stop();

        Intake.spin(fwd,-100,percent);
        wait(1000,msec);
        Chassis.right_swing_to_angle(70,12,1,1,0,.25,.015,1.1,15);
        catapultShoot();
             if(Loads == TargetLoads)
             break;
        // moveToPosition(-135,-135,225,false,80,70); blue
        Chassis.turn_to_angle(45);
        //Chassis.drive_distance(5);
        //wait(700,msec);
        //if((abs(GPS.xPosition(mm))) < 100 || (abs(GPS.xPosition(mm))) > 150 || (abs(GPS.yPosition(mm))) < 100|| (abs(GPS.yPosition(mm))) > 150);
        //moveToPosition(130,130,45,false,50,50);

        LeftDriveSmart.spin(fwd,10,percent);
        RightDriveSmart.spin(fwd,10,percent);
        Loads += 1;
        fprintf(fp,"Lanched: %.1f", Loads);
      } 
      Intake.stop();
      LeftDriveSmart.stop();
      RightDriveSmart.stop();

  }


  

  void ImproSwing(int LVel, int RVel, int Deg)
  {
    LeftDriveSmart.resetPosition();
    RightDriveSmart.resetPosition();
    Chassis.drive_with_voltage(LVel*0.12,RVel*0.12);
    //LeftDriveSmart.spin(fwd,LVel,percent);
    //RightDriveSmart.spin(fwd,RVel,percent);
    waitUntil(abs(LeftDriveSmart.position(degrees)) >= Deg || abs(RightDriveSmart.position(degrees)) >= Deg );
    LeftDriveSmart.stop();
    RightDriveSmart.stop();
  }



  void Hanging()
  {
  //leftWings.set(false);
  //rightWings.set(false);
  moveToPosition(100,130,90,false,100,100);
  ratchet.set(false);
  checkPosition(HangPos);
  Hang.spinFor(fwd,HangTurns,degrees,false);

    //ImproSwing(-20,-100,750);
  Chassis.drive_distance(-17);
  waitUntil(hangEncoder.angle(degrees) > HangPos-5 && hangEncoder.angle(degrees) < HangPos+5);
  Chassis.drive_with_voltage(-12,-12);
  checkPosition(200);
  Hang.spinFor(fwd,HangTurns,degrees);
  ratchet.set(true);
  checkPosition(20);
  Hang.spinFor(fwd,HangTurns,degrees);
  
  Chassis.drive_with_voltage(0,0);

}



  void ShootingRoutine(int TargetLoads)
  {
    startSubsystems();
    ScoreAllianceTriball();
    runMatchload(20);
    
  }
  
  void ShootingRoutineInteraction(int TargetLoads)
  {
    runMatchload(11);
    Hanging();
  }

}