/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       student                                                   */
/*    Created:      9/9/2024, 5:11:30 PM                                      */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain; 
controller Controller1;

motor LF (PORT3, ratio6_1, true);
motor LM (PORT5, ratio6_1, true);
motor LB (PORT17, ratio6_1, true);

motor RF (PORT12, ratio6_1, false);
motor RM (PORT6, ratio6_1, false);
motor RB (PORT13, ratio6_1, false);

motor Intake (PORT14, ratio18_1, false);
motor Lifter (PORT18, ratio6_1, true);

digital_out Clampy = digital_out(Brain.ThreeWirePort.A);
/*---------------------------------------------------------------------------*/

 void drive (int rspeed, int lspeed, int wt){                      
  LF.spin(fwd, lspeed, pct);
  LM.spin(fwd, lspeed, pct);  
  LB.spin(fwd, lspeed, pct);

  RF.spin(fwd, rspeed, pct);  
  RM.spin(fwd, rspeed, pct);  
  RB.spin(fwd, rspeed, pct);

  wait (wt, msec);        
                                                         
}

void useless(){ 
  
}

void inchDrive(){
  
}

  double YOFFSET = 20; //offset for the display
//Writes a line for the diagnostics of a motor on the Brain
void MotorDisplay(double y, double curr, double temp)
{
Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(5, YOFFSET + y, "Current: %.1fA", curr);
if (curr < 1)
Brain.Screen.setFillColor(green);
else if (curr >= 1 && curr  <= 2.5)
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(140, YOFFSET + y - 15, 15, 15);

Brain.Screen.setFillColor(transparent);
Brain.Screen.printAt(160, YOFFSET + y, "Temp: %.1fC", temp);
if (temp < 45)
Brain.Screen.setFillColor(green);
else if (temp <= 50 && temp  >= 45)
// TRUE and TRUE --> True
// TRUE and FALSE --> False
// FALSE and FALSE --> False
Brain.Screen.setFillColor(yellow);
else
Brain.Screen.setFillColor(red);
Brain.Screen.drawRectangle(275, YOFFSET + y - 15, 15, 15);
Brain.Screen.setFillColor(transparent);
}


//Displays information on the brain
void Display()
{
double leftFrontCurr = LF.current(amp);
double leftFrontTemp = LF.temperature(celsius);
double leftBackCurr = LB.current(amp);
double leftBackTemp = LB.temperature(celsius);
double rightFrontCurr = RF.current(amp);
double rightFrontTemp = RF.temperature(celsius);
double rightBackCurr = RB.current(amp);
double rightBackTemp = RB.temperature(celsius);
double rightMiddleCurr = RM.current(amp);
double rightMiddleTemp = RM.temperature(celsius);
double leftMiddleCurr = LM.current(amp);
double leftMiddleTemp = RM.temperature(celsius);


if (LF.installed())
{
MotorDisplay(1, leftFrontCurr, leftFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 1, "LeftFront");
}
else
Brain.Screen.printAt(5, YOFFSET + 1, "LeftFront Problem");

if (LB.installed())
{
MotorDisplay(31, leftBackCurr, leftBackTemp);
Brain.Screen.printAt(300, YOFFSET + 31, "LeftBack");
}

else
Brain.Screen.printAt(5, YOFFSET + 31, "LeftBack Problem");

if (RF.installed())
{
MotorDisplay(61, rightFrontCurr, rightFrontTemp);
Brain.Screen.printAt(300, YOFFSET + 61, "RightFront");
}
else
Brain.Screen.printAt(5, YOFFSET + 61, "RightFront Problem");

if (RB.installed())
{
MotorDisplay(91, rightBackCurr, rightBackTemp);
Brain.Screen.printAt(300, YOFFSET + 91, "RightBack");
}
else
Brain.Screen.printAt(5, YOFFSET + 91, "RightBack Problem");

if (RM.installed())
{
MotorDisplay(121, rightMiddleCurr, rightMiddleTemp);
Brain.Screen.printAt(300, YOFFSET + 121, "RightMiddle");
}
else
Brain.Screen.printAt(5, YOFFSET + 121, "RightMiddle Problem");

if (LM.installed())
{
MotorDisplay(151, leftMiddleCurr, leftMiddleTemp);
Brain.Screen.printAt(300, YOFFSET + 151, "LeftMiddle");
}
else
Brain.Screen.printAt(5, YOFFSET + 151, "LeftMiddle Problem");
}

void clampPush(bool push){
  Clampy.set(push);
}


/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    Display();
    
    int lspeed = Controller1.Axis3.position(pct);
    int rspeed = Controller1.Axis2.position(pct);

    drive(rspeed, lspeed, 10);

    if (Controller1.ButtonA.pressing()){
    clampPush(true);
    }
    else if (Controller1.ButtonB.pressing()){
    clampPush(false);
    }

    if (Controller1.ButtonR1.pressing()){
      Intake.spin(fwd, 100, pct);
      Lifter.spin(fwd,50, pct);
    }
    else if (Controller1.ButtonR2.pressing()){
      Intake.spin(reverse,100, pct);
      Lifter.spin(reverse,50, pct);
    }
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
