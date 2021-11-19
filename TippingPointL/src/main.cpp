/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Robotics                                         */
/*    Created:      Fri Aug 06 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;

double x = 0;
double y = 0;

competition Competition;
controller Controller1;
controller Controller2;

//T = top B = bottom
motor TLeft(PORT12, ratio18_1);
motor BLeft(PORT20, ratio18_1);
motor TRight(PORT8, ratio18_1, true);
motor BRight(PORT9, ratio18_1, true);
motor_group Left(TLeft, BLeft);
motor_group Right(TRight, BRight);
motor_group DTrain(TLeft, BLeft, TRight, BRight);
motor Intake(PORT10, ratio18_1,true);
motor Lift(PORT4, ratio36_1);

gps GPS(PORT20); //this thing is actually so cool
distance Distance(PORT12);
inertial Inertia(PORT20);

//Switch between 2 different controllers for driver control (refer to the jank function graveyard)
controller CurDrive = Controller1;
int vCurDrive = 1;

//mecanum wheel strafing. 0 = left 1 = right (works in both driver and auton)
void strafe(int direct, int speed){
  if(direct == 0){
    TLeft.spin(reverse, speed, pct);
    BLeft.spin(fwd, speed, pct);
    TRight.spin(fwd, speed, pct);
    BRight.spin(reverse, speed, pct);
  }
  else if(direct == 1){
    TLeft.spin(fwd, speed, pct);
    BLeft.spin(reverse, speed, pct);
    TRight.spin(reverse, speed, pct);
    BRight.spin(fwd, speed, pct);
  }
}

void auton(){ //rough draft
Intake.spin(fwd, 72.5, pct); //drop rings
wait(3, sec);
Intake.stop(hold);
wait(0.5, sec);
DTrain.spin(reverse, 80, pct); //get ready to steal goal
wait(0.2, sec);
strafe(0, 80);
wait(0.15, sec);
Left.spin(fwd, 80, pct);
Right.spin(reverse, 80, pct);
wait(0.45, sec);
DTrain.stop(coast);
wait(0.3, sec);
Lift.spin(fwd, 75, pct); //shake the lift
wait(0.6, sec);
Lift.spin(reverse, 75, pct);
wait(0.6, sec);
Lift.stop(hold);
}

void driver(){
  int t = 0;
  while(1){
    //strafing (using 3D printed shoulder thingies)
    if(CurDrive.ButtonDown.pressing()){
      strafe(0, 75);
    }
    else if(CurDrive.ButtonB.pressing()){
      strafe(1, 75);
    }
    //drivetrain (tank)
    else{
    Left.spin(fwd, CurDrive.Axis3.position(), pct);
    Right.spin(fwd, CurDrive.Axis2.position(), pct);
    }

    //intake
    if(CurDrive.ButtonL1.pressing()){
      Intake.spin(fwd, 92.5, pct);
      t = 1;
    }
    else if(CurDrive.ButtonL2.pressing()){
      Intake.spin(reverse, 92.5, pct);
    }
    else{
      Intake.stop(hold);
    }

    //lift
    if(CurDrive.ButtonR1.pressing()){
      Lift.spin(fwd, 25, pct);
    }
    else if(CurDrive.ButtonR2.pressing()){
      Lift.spin(reverse, 25, pct);
    }
    else{
      Lift.stop(hold);
    }

    //CurDrive.ButtonDown.pressed(ControllerSwitch);
  }
}

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  GPS.calibrate();
  Inertia.calibrate();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    Brain.Screen.printAt(20,20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Lift Motor Pos: %f", Lift.position(degrees));
  }
}

//the jank function graveyard (nonfunctional functions that i hate scrolling past)
/*void ControllerSwitch(){
  std::cout << "PRESSED!";
  if(int vCurDrive = 1){
      Controller1.Screen.clearLine(3);
      controller CurDrive = Controller2;
      vCurDrive = 2;
      CurDrive.Screen.clearLine(3);
      CurDrive.Screen.print("DRIVING");
    }
  else if(int vCurDrive = 2){
      Controller2.Screen.clearLine(3);
      controller CurDrive = Controller1;
      vCurDrive = 1;
      CurDrive.Screen.clearLine(3);
      CurDrive.Screen.print("DRIVING");
    }
}*/ 