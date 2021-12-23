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
#include <math.h>

using namespace vex;

double shift = 1;
double toDegrees = 180/M_PI;

//define competition functions/controllers
competition Competition;
controller Controller1;
controller Controller2;

//define motors
motor Left1(PORT18, ratio18_1, true);
motor Left2(PORT10 , ratio18_1, true);
motor Left3(PORT8, ratio18_1, true);
motor Right1(PORT14, ratio18_1, true);
motor Right2(PORT20, ratio18_1, true);
motor Right3(PORT13, ratio18_1, true);
motor_group Left(Left1, Left2, Left3);
motor_group Right(Right1, Right2, Right3);
motor_group DTrain(Left1, Left2, Left3, Right1, Right2, Right3);
motor IL(PORT21, ratio18_1,true);
motor Hook(PORT16);

//define sensors
gps GPS(PORT7);
distance DistanceL(PORT9);
distance DistanceR(PORT19);
inertial Inertia(PORT8);

//Switch between 2 different controllers for driver control (refer to the jank function graveyard)
controller CurDrive = Controller1;
int vCurDrive = 1;

//gear shift task
int gearShift(){
  CurDrive.Screen.setCursor(3, 1);
  CurDrive.Screen.print("MAX DUMPY");
  while(1){
    if(CurDrive.ButtonB.pressing()){
      CurDrive.Screen.clearLine(3);
      wait(0.2, sec); //this is why it's a task btw
      if(shift == 1){
        shift = 0.5;
        CurDrive.Screen.print("50%% DUMPY");
      }
      else if(shift == 0.5){
        shift = 0.1;
        CurDrive.Screen.print("10%% DUMPY");
      }
      else if(shift == 0.1){
        shift = 1;
        CurDrive.Screen.print("MAX DUMPY");
      }
    }
  }
  return 1;
}

//task for toggling solenoids
void toggleSolenoid(controller::button butt, pneumatics solenoid){
  while(1){
    if(butt.pressing()){
      if(solenoid.value() == 0){
        solenoid.open();
        wait(0.2, sec);
      }
      else if(solenoid.value() == 1){
        solenoid.close();
        wait(0.2, sec);
      }
    }
  }
}
int toggleLeftClamp(){ //tasks don't take parameters :(
  toggleSolenoid(CurDrive.ButtonRight, LeftClamp);
  return 0;
}
int toggleRightClamp(){
  toggleSolenoid(CurDrive.ButtonY, RightClamp);
  return 0;
}

//WE'VE SWITCHED TO OMNI WHEELS SO THIS IS NO LONGER BEING USED (keeping just in case)
//mecanum wheel strafing. 0 = left 1 = right (works in both driver and auton)
/*void strafe(int direct, int speed){
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
}*/

//spinFor but with speed to make autonomous less of a pain
//i have learned that time-based spinFor actually uses speed but rotation-based doesn't???? ok
void speedFor(motor Motor, directionType direct, double rotations, double speed, bool waitfor=true){
  Motor.setVelocity(speed, pct);
  Motor.spinFor(direct, rotations, rev, waitfor);
}

//same thing but for motor groups
void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed, bool waitfor=true){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.spinFor(direct, rotations, rev, waitfor);
}

//prototype (i have no idea if this works)
void moveTo(double xFinal, double yFinal){
  double deltaX = xFinal - GPS.xPosition();
  double deltaY = yFinal - GPS.yPosition();
  double turnAngle = atan(fabs(deltaY)/fabs(deltaX))*toDegrees; //trigonometry!
  std::cout << turnAngle;
}

//outdated, needs to be redone. completely.
void auton(){
  //Hook.spinFor(reverse, 3.6, rev, false);
  DTrain.spinFor(reverse, 1.5, sec, 80, velocityUnits::pct); //go to get neutral goal
  //Hook.spinFor(fwd, 3, rev, false);
  DTrain.spin(reverse, 80, pct);
  wait(0.3, sec);
  DTrain.stop(hold);
  DTrain.spin(forward, 60, pct); //after picking up neutral, go back
  wait(1.73, sec);
  DTrain.stop(hold);
  speedForGroup(Left, reverse, 0.79, 25, false);
  speedForGroup(Right, fwd, 0.79, 25);
  DTrain.stop(hold);
  speedForGroup(DTrain, reverse, 0.5, 60, false);
  speedFor(IL, fwd, 1.35 ,50);
  DTrain.spin(fwd, 60, pct);
  wait(1.35, sec);
  speedForGroup(DTrain, reverse, 0.3, 20, false);
  speedFor(IL, reverse, 0.6175, 50, false);
  wait(0.5, sec);
  DTrain.stop(hold);
  speedForGroup(DTrain, reverse, 1.3, 50,false);
  wait(1.2, sec);
  DTrain.stop(hold);
  //Intake.spinFor(fwd, 10000, rev, false); 
  /*speedForGroup(DTrain, fwd, 2, 30);
  DTrain.stop(hold);
  wait(0.85, sec);
  speedForGroup(DTrain, reverse, 2, 50);
  DTrain.stop(hold);
  speedForGroup(DTrain, fwd, 2, 30);*/
}

void driver(){
  task jank(gearShift);
  //task jank2(toggleLeftClamp);
  //task jank3(toggleRightClamp);
  while(1){
    //drivetrain (tank)
     if(CurDrive.ButtonDown.pressing()){
      DTrain.stop(hold);
    }
    else{
    //drivetrain (tank) (if strafe() returns make this an else statement)
    Left1.spin(reverse, CurDrive.Axis3.position()*shift, pct);
    Left2.spin(fwd, CurDrive.Axis3.position()*shift, pct);
    Left3.spin(fwd, CurDrive.Axis3.position()*shift, pct);
    Right1.spin(fwd, CurDrive.Axis2.position()*shift, pct);
    Right2.spin(reverse, CurDrive.Axis2.position()*shift, pct);
    Right3.spin(reverse, CurDrive.Axis2.position()*shift, pct);
    }

    if(CurDrive.ButtonL1.pressing()){
      IL.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonR1.pressing()){
      IL.spin(reverse, 65, pct);
    }
    else{
      IL.stop(coast);
    }

    if(CurDrive.ButtonL2.pressing()){
      RRelease.open();
    }
    else{
      RRelease.close();
    }

    if(CurDrive.ButtonR2.pressing()){
      Hook.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonA.pressing()){
      Hook.spin(reverse, 100, pct);
    }
    else{
      Hook.stop(hold);
    }

    if(CurDrive.ButtonRight.pressing()){
      LeftClamp.open();
    }
    else{
      LeftClamp.close();
    }

    if(CurDrive.ButtonA.pressing()){
      RightClamp.open();
    }
    else{
      RightClamp.close();
    }
  }
}

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  GPS.calibrate();
  Inertia.calibrate();
  IL.resetPosition();
  RRelease.close();
  RightClamp.close();
  LeftClamp.close();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    Brain.Screen.printAt(20, 20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Left1 Temp: %f ℃", Left1.temperature(celsius));
    Brain.Screen.printAt(20, 60, "Left2 Temp: %f ℃", Left2.temperature(celsius));
    Brain.Screen.printAt(20, 80, "Left3 Temp: %f ℃", Left3.temperature(celsius));
    Brain.Screen.printAt(20, 100, "Right1 Temp: %f ℃", Right1.temperature(celsius));
    Brain.Screen.printAt(20, 120, "Right2 Temp: %f ℃", Right2.temperature(celsius));
    Brain.Screen.printAt(20, 140, "Right3 Temp: %f ℃", Right3.temperature(celsius));
  }
}