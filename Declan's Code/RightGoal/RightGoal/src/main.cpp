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

motor Left1(PORT19, ratio18_1, true);
motor Left2(PORT17 , ratio18_1, true);
motor Left3(PORT18, ratio18_1, true);
motor Right1(PORT16, ratio18_1, true);
motor Right2(PORT11, ratio18_1, true);
motor Right3(PORT15, ratio18_1, true);
motor_group Left(Left1, Left2, Left3);
motor_group Right(Right1, Right2, Right3);
motor_group DTrain(Left1, Left2, Left3, Right1, Right2, Right3);
motor IL(PORT3, ratio18_1,true);


gps GPS(PORT7);
distance DistanceL(PORT9);
distance DistanceR(PORT19);
inertial Inertia(PORT8);

//Switch between 2 different controllers for driver control (refer to the jank function graveyard)
controller CurDrive = Controller1;
int vCurDrive = 1;

//moves lift. 0 = down 1 = up
void movelift(int direct, double speed){
  if(direct == 0){
    IL.spin(fwd, speed, pct);
  }
  else if(direct == 1 and DistanceR.objectDistance(mm) > 253){
    IL.spin(reverse, speed, pct);
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

//rotations to neutral = 3.888
void auton(){

}

void driver(){
  int t = 0;
  while(1){
    
    //drivetrain (tank)
     if(CurDrive.ButtonDown.pressing()){
      DTrain.stop(hold);
    }
    else if(CurDrive.ButtonB.pressing()){
      Left.spin(fwd, CurDrive.Axis3.position()*0.1, pct);
      Right.spin(fwd, CurDrive.Axis2.position()*0.1, pct);
    }
    else{
    //drivetrain (tank) (if strafe() returns make this an else statement)
    Left1.spin(reverse, CurDrive.Axis3.position(), pct);
    Left2.spin(fwd, CurDrive.Axis3.position(), pct);
    Left3.spin(fwd, CurDrive.Axis3.position(), pct);
    Right1.spin(fwd, CurDrive.Axis2.position(), pct);
    Right2.spin(reverse, CurDrive.Axis2.position(), pct);
    Right3.spin(reverse, CurDrive.Axis2.position(), pct);
    }

    if(CurDrive.ButtonL1.pressing()){
      IL.spin(fwd, 100, pct);
      t = 1;
    }
    else if(CurDrive.ButtonR1.pressing()){
      IL.spin(reverse, 65, pct);
    }
    else{
      IL.stop(coast);
    }
  }
}


int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  GPS.calibrate();
  Inertia.calibrate();
  IL.resetPosition();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    avgdistance = (DistanceL.objectDistance(mm) + DistanceR.objectDistance(mm)) / 2;
    Brain.Screen.printAt(20, 20, "Gyro: %f", Inertia.heading());
    Brain.Screen.printAt(20, 40, "Left Distance: %f mm", DistanceL.objectDistance(mm));
    Brain.Screen.printAt(20, 60, "Right Distance: %f mm", DistanceR.objectDistance(mm));
    Brain.Screen.printAt(20, 80, "Average Distance: %f mm", avgdistance);
    Brain.Screen.printAt(20, 100, "TLeft Temp: %f ℃", Left1.temperature(celsius));
    Brain.Screen.printAt(20, 120, "TRight Temp: %f ℃", Right1.temperature(celsius));
    Brain.Screen.printAt(20, 140, "BLeft Temp: %f ℃", Left2.temperature(celsius));
    Brain.Screen.printAt(20, 160, "BRight Temp: %f ℃", Right2.temperature(celsius));
    Brain.Screen.printAt(20, 180, "Lift Temp: %f ℃", IL.temperature(celsius));
    Brain.Screen.printAt(20, 200, "Hook Temp: %f ℃", Right3.temperature(celsius));
     Brain.Screen.printAt(20, 200, "Hook Temp: %f ℃", Left3.temperature(celsius));
    Brain.Screen.printAt(20, 200, "Intake Current: %f A", IL.current());
    
  }
}
