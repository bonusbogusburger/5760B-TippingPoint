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
int goaldetect = 1;
competition Competition;
controller Controller1;
controller Controller2;

//T = top/front B = bottom/back
motor TLeft(PORT16, ratio18_1);
motor BLeft(PORT20 , ratio18_1);
motor TRight(PORT6, ratio18_1, true);
motor BRight(PORT12, ratio18_1, true);
motor_group Left(TLeft, BLeft);
motor_group Right(TRight, BRight);
motor_group DTrain(TLeft, BLeft, TRight, BRight);
motor Intake(PORT14, ratio18_1,true);
motor Lift(PORT4, ratio36_1);
motor Hook(PORT18, ratio18_1);

gps GPS(PORT20); //this thing is actually so cool
distance Distance(PORT13);
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

//moves lift. 0 = down 1 = up
void movelift(int direct, double speed){
  if(direct == 0){
    Lift.spin(fwd, speed, pct);
  }
  else if(direct == 1 and Distance.objectDistance(mm) > 253){
    Lift.spin(reverse, speed, pct);
  }
}

void auton(){ // testing encoders
  Hook.spinFor(reverse, 5.658, rev, false);
  DTrain.spinFor(reverse, 4.05, rev);
  //Hook.spinFor(fwd, 3, rev);
  //DTrain.spinFor(fwd, 2, rev);
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
      Intake.spin(fwd, 80, pct);
      t = 1;
    }
    else if(CurDrive.ButtonL2.pressing()){
      Intake.spin(reverse, 80, pct);
    }
    else if(CurDrive.ButtonLeft.pressing()){
      Intake.spin(fwd, 20, pct);
    }
    else{
      Intake.stop(hold);
    }

    //lift
    if(CurDrive.ButtonR2.pressing()){
      Lift.spin(fwd, 50, pct);
    }
    else if(CurDrive.ButtonR1.pressing()){
      Lift.spin(reverse, 25, pct);
    }
    else{
      Lift.stop(hold);
    }

    //hook
    if(CurDrive.ButtonY.pressing()){
      Hook.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonRight.pressing()){
      Hook.spin(reverse, 100, pct);
    }
    else{
      Hook.stop(hold);
    }

    //brake drivetrain motors
    if(CurDrive.ButtonUp.pressing()){
      DTrain.stop(hold);
    }

    //CurDrive.ButtonA.pressed(auton);
  }
}

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  GPS.calibrate();
  Inertia.calibrate();
  Hook.setVelocity(100, pct);
  DTrain.setVelocity(60, pct);
  Lift.setPosition(-475, deg); //bot starts with lift upwards at -945 degrees
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    Brain.Screen.printAt(20,20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Object Distance: %f mm", Distance.objectDistance(mm));
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