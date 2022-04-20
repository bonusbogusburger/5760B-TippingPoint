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
  else if(direct == 1 and DistanceR.objectDistance(mm) > 253){
    Lift.spin(reverse, speed, pct);
  }
}

//spinFor but with speed to make autonomous less of a pain
void speedFor(motor Motor, directionType direct, double rotations, double speed){
  Motor.setVelocity(speed, pct);
  Motor.spinFor(direct, rotations, rev);
}

void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.spinFor(direct, rotations, rev);
}

void correctDrive(directionType direct, double speed){
  while(DistanceL.objectDistance(mm) != DistanceR.objectDistance(mm)){
    if(DistanceL.objectDistance(mm) > DistanceR.objectDistance(mm)){
      Left.spin(direct, speed*0.9, pct);
      Right.spin(direct, speed, pct);
    }
    else if(DistanceR.objectDistance(mm) > DistanceL.objectDistance(mm)){
      Right.spin(direct, speed*0.9, pct);
      Left.spin(direct, speed, pct);
    }
  }
  DTrain.spin(direct, speed, pct);
}

void correctDriveGyro(directionType direct, double speed){
  if(Inertia.heading() < 0){
    Left.spin(direct, speed*0.9, pct);
    Right.spin(direct, speed, pct);
  }
  else if(Inertia.heading() > 0){
    Right.spin(direct, speed*0.9, pct);
    Left.spin(direct, speed, pct);
  }
}

void auton(){ //rough draft
  DTrain.spin(fwd, 30, pct); //push goal
  wait(1.4, sec);
  DTrain.stop(coast);
  wait(0.5, sec);
  Intake.spin(fwd, 75, pct); //drop rings
  wait(3, sec);
  Intake.stop(hold);
  DTrain.spin(reverse, 50, pct); //back tf up
  wait(1.4, sec);
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
    avgdistance = (DistanceL.objectDistance(mm) + DistanceR.objectDistance(mm)) / 2;
    Brain.Screen.printAt(20, 20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Left Distance: %f mm", DistanceL.objectDistance(mm));
    Brain.Screen.printAt(20, 60, "Right Distance: %f mm", DistanceR.objectDistance(mm));
    Brain.Screen.printAt(20, 80, "Average Distance: %f mm", avgdistance);
    Brain.Screen.printAt(20, 100, "TLeft Temp: %f ℃", TLeft.temperature(celsius));
    Brain.Screen.printAt(20, 120, "TRight Temp: %f ℃", TRight.temperature(celsius));
    Brain.Screen.printAt(20, 140, "BLeft Temp: %f ℃", BLeft.temperature(celsius));
    Brain.Screen.printAt(20, 160, "BRight Temp: %f ℃", BRight.temperature(celsius));
    Brain.Screen.printAt(20, 180, "Arm Temp: %f ℃", Lift.temperature(celsius));
    Brain.Screen.printAt(20, 200, "Hook Temp: %f ℃", Hook.temperature(celsius));
    
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