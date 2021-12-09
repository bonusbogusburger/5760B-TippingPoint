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
double avgdistance = 0;
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

gps GPS(PORT7);
distance DistanceL(PORT9);
distance DistanceR(PORT19);
inertial Inertia(PORT8);

//Switch between 2 different controllers for driver control (refer to the jank function graveyard)
controller CurDrive = Controller1;
int vCurDrive = 1;

//mecanum wheel strafing. 0 = left 1 = right (works in both driver and auton)
void strafe(int direct, int speed){
  if(direct == 0){
    TLeft.spin(reverse, speed*0.95, pct);
    BLeft.spin(fwd, speed*0.95, pct);
    TRight.spin(fwd, speed, pct);
    BRight.spin(reverse, speed, pct);
  }
  else if(direct == 1){
    TLeft.spin(fwd, speed*0.95, pct);
    BLeft.spin(reverse, speed*0.95, pct);
    TRight.spin(reverse, speed, pct);
    BRight.spin(fwd, speed, pct);
  }
}

//moves lift. 0 = down 1 = up
void movelift(int direct, double speed){
  if(direct == 0){
    Lift.spin(fwd, speed, pct);
  }
  else if(direct == 1 and DistanceR.objectDistance(mm) > 253){
    Lift.spin(reverse, speed, pct);
  }
}

//spinFor but with speed to make autonomous less of a pain
void speedFor(motor Motor, directionType direct, double rotations, double speed, bool waitfor=true){
  Motor.setVelocity(speed, pct);
  Motor.spinFor(direct, rotations, rev, waitfor);
}

void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed, bool waitfor=true){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.spinFor(direct, rotations, rev, waitfor);
}

void correctDrive(directionType direct, double speed){
 while(DistanceL.objectDistance(mm) != DistanceR.objectDistance(mm)){
    if(direct == reverse){
      if(DistanceL.objectDistance(mm) > DistanceR.objectDistance(mm)){
        Left.spin(direct, speed*0.9, pct);
        Right.spin(direct, speed, pct);
      }
      else if(DistanceR.objectDistance(mm) > DistanceL.objectDistance(mm)){
        Right.spin(direct, speed*0.9, pct);
        Left.spin(direct, speed, pct);
      }
    }
    if(direct == fwd){
      if(DistanceL.objectDistance(mm) < DistanceR.objectDistance(mm)){
        Left.spin(direct, speed*0.9, pct);
        Right.spin(direct, speed, pct);
      }
      else if(DistanceR.objectDistance(mm) < DistanceL.objectDistance(mm)){
        Right.spin(direct, speed*0.9, pct);
        Left.spin(direct, speed, pct);
      }
    }
  }
  DTrain.spin(direct, speed, pct);
}

void turnBot(int ninetydegturns){
return;
}

//rotations to neutral = 3.888
void auton(){ //this hurts to look at but it works (hopefully) so shush
  Hook.spinFor(reverse, 3.6, rev, false);
  DTrain.spin(reverse, 80, pct);
  wait(1.5, sec);
  Hook.spinFor(fwd, 3.6 , rev, false);
  DTrain.spin(reverse, 80, pct);
  wait(0.3, sec);
  DTrain.spin(forward, 60, pct);
  wait(1.5, sec);
  speedForGroup(Right, fwd, 1.6, 50, false);
  speedForGroup(Left, reverse, 1.6, 50);
  speedForGroup(DTrain, reverse, 0.5, 60, false);
  speedFor(Lift, fwd, 1.35 ,50);
  DTrain.spin(fwd, 60, pct);
  wait(1.35, sec);
  speedForGroup(DTrain, reverse, 0.3, 20, false);
  speedFor(Lift, reverse, 0.6175, 50, false);
  wait(0.5, sec);
  speedForGroup(DTrain, reverse, 1.3, 50,false);
  wait(1.2, sec);
  speedForGroup(DTrain, fwd, 0.425, 25, false);
  speedForGroup(Left, fwd, 0.7, 50, false);
  speedForGroup(Right, reverse, 0.7, 50);
  speedForGroup(DTrain, fwd, 1.8, 30, false);
  Intake.spin(fwd, 85, pct);
  wait(4, sec);
  Intake.stop(coast);
  speedForGroup(Right, reverse, 0.125, 60);
  DTrain.spin(reverse, 80, pct);
  wait(3.25, sec);
  DTrain.stop(coast);
  wait(0.25, sec);
  DTrain.spin(forward, 50, pct);
  wait(0.25, sec);    
  speedForGroup(Left, reverse, 2.5 , 50, false);
  speedForGroup(Right, fwd, 2.5 , 50);
  DTrain.spin(forward, 50, pct);
  wait(0.1, sec);
  speedFor(Lift, forward, 0.6175, 50);
  DTrain.spin(reverse, 80, pct);
  wait(0.6, sec);
  strafe(0, 80);
  wait(0.5, sec);
  DTrain.spin(reverse, 50, pct);
  wait(2.75, sec);
  speedForGroup(DTrain, fwd, 0.15, 60, false);
  strafe(0, 80);
  wait(1.3, sec);
  DTrain.spin(fwd, 80, pct);
  wait(1.65, sec);
  speedFor(Lift, reverse, 0.2, 50);
  DTrain.spin(reverse, 80, pct);
  wait(3, sec);
  DTrain.stop(coast);
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

    //brake drivetrain motors (seems to make all motors sticky after being held for a reason beyond my comprehension)
    if(CurDrive.ButtonUp.pressing()){
      DTrain.stop(hold);
    }

    //intake
    if(CurDrive.ButtonL1.pressing()){
      Intake.spin(fwd, 100, pct);
      t = 1;
    }
    else if(CurDrive.ButtonL2.pressing()){
      Intake.spin(reverse, 100, pct);
    }
    else if(CurDrive.ButtonLeft.pressing()){
      Intake.spin(fwd, 20, pct);
    }
    else{
      Intake.stop(hold);
    }

    //lift
    if(CurDrive.ButtonR2.pressing()){
      Lift.spin(fwd, 65, pct);
    }
    else if(CurDrive.ButtonR1.pressing()){
      Lift.spin(reverse, 50, pct);
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

    //CurDrive.ButtonA.pressed(auton);
  }
}

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  GPS.calibrate();
  Inertia.calibrate();
  Hook.setVelocity(100, pct);
  Lift.resetPosition();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    avgdistance = (DistanceL.objectDistance(mm) + DistanceR.objectDistance(mm)) / 2;
    Brain.Screen.printAt(20, 20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Left Distance: %f mm", DistanceL.objectDistance(mm));
    Brain.Screen.printAt(20, 60, "Right Distance: %f mm", DistanceR.objectDistance(mm));
    Brain.Screen.printAt(20, 80, "Average Distance: %f mm", avgdistance);
    Brain.Screen.printAt(20, 100, "TLeft Temp: %f ℃", TLeft.temperature(celsius));
    Brain.Screen.printAt(20, 120, "TRight Temp: %f ℃", TRight.temperature(celsius));
    Brain.Screen.printAt(20, 140, "BLeft Temp: %f ℃", BLeft.temperature(celsius));
    Brain.Screen.printAt(20, 160, "BRight Temp: %f ℃", BRight.temperature(celsius));
    Brain.Screen.printAt(20, 180, "Lift Temp: %f ℃", Lift.temperature(celsius));
    Brain.Screen.printAt(20, 200, "Hook Temp: %f ℃", Hook.temperature(celsius));
    Brain.Screen.printAt(20, 200, "Intake Current: %f A", Intake.current());
    
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