/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Robotics                                         */
/*    Created:      Thu Mar 17 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

//Welcome to T.R.I.S.T.A.N.2, Worlds Boogaloo
#include "vex.h"
#include <iostream>
#include <math.h>
#include <ctime>

using namespace vex;

//define competition functions/controllers
competition Competition;
controller Controller;

//define motors
motor Left1(PORT5, ratio18_1, false);
motor Left2(PORT6 , ratio18_1, true);
motor Left3(PORT10, ratio18_1, true);  //orignally port 17
motor Right1(PORT14, ratio18_1, true);
motor Right2(PORT20, ratio18_1, false);
motor Right3(PORT13, ratio18_1, false);
motor_group Left(Left1, Left2, Left3);
motor_group Right(Right1, Right2, Right3);
drivetrain DTrain(Left, Right);
motor_group GDTrain(Left1, Left2, Left3, Right1, Right2, Right3);
motor IL(PORT21, ratio18_1,true);
motor Hook(PORT16);
motor Motor(PORT17);

//define sensors
gps GPS(PORT9);
distance DistanceL(PORT18);
distance DistanceR(PORT19);
inertial Inertial(PORT2);
vision Vision(PORT17);
triport ThreeWirePort = vex::triport( vex::PORT22 );
vex::limit HookLimit = vex::limit(ThreeWirePort.A);

double speeds [10] = {1.00, .90, .80, .70, .60, .50, .40, .30, .20, .10};
double speed = 0.3;
int speedSwitcher(){ //task that switches speed hallelujah
  return 1;
}

void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed, bool waitfor=true){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.resetPosition();
  MotorGroup.spin(fwd);
  waitUntil(fabs(MotorGroup.position(rev)) >= fabs(rotations));
  //MotorGroup.spinFor(direct, rotations, rev, waitfor);
}

double stickTest3;
double stickTest1;
int timer3 = 0;
int timediff = 0;
int stopTimer = 0;
double lastStick3;
double lastStick1;
int msec3(){
  while(floor(Controller.Axis3.position()/10)*10 != 0){
    wait(1, msec);
    timer3 += 1;
    timediff += 1;
  }
  return 1;
}
int stick3time(){ //timing method is inaccurate, probably won't use
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  task m3(msec3);
  while(floor(Controller.Axis3.position()/10)*10 != 0){
    stickTest3 = floor(Controller.Axis3.position()/10)*10;
    double mod = Controller.Axis1.position()*0.65;
    Left.spin(fwd, stickTest3+mod, pct);
    Right.spin(fwd, stickTest3-mod, pct);
    if(stickTest3 == 0){
      void;
    }
    else if(lastStick3 != stickTest3){
      std::cout.precision(3); //the next line has to be very long i am so sorry
      //std::cout << "Accelerated from " << lastStick3 << " to " << stickTest3 << " (GPS X: " << GPS.xPosition(inches) << " Y: " << GPS.yPosition(inches) << ") " << timediff << "ms " << timer3 << "ms" << std::endl;
      std::cout << "wait(" << timediff << ", msec);" << std::endl;
      std::cout << "DTrain.drive(fwd, " << stickTest3 << ", velocityUnits::pct);" << std::endl;
      timediff = 0;
    }
    lastStick3 = stickTest3;
  }
  std::cout << "wait(" << timediff << ", msec);" << std::endl;
  std::cout << "DTrain.stop(brake);" << std::endl;
  return 1;
}
int stick3rev(){ //pretty accurate but for slow accelerations, things start to get wonky. can likely be tuned to become accurate
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  stopTimer = 0;
  Left.resetPosition();
  Right.resetPosition();
  while(floor(Controller.Axis3.position()/10)*10 != 0){
    stickTest3 = floor(Controller.Axis3.position()/10)*10;
    double mod = Controller.Axis1.position()*0.65;
    Left.spin(fwd, stickTest3+mod, pct);
    Right.spin(fwd, stickTest3-mod, pct);
    if(stickTest3 == 0){
      void;
    }
    else if(lastStick3 != stickTest3){
      std::cout.precision(5); 
      double DTrainrev = (Left.position(rev)+Right.position(rev))/2; 
      std::cout << "speedForGroup(GDTrain, fwd, " << DTrainrev << ", " << lastStick3 << ");" << std::endl;
      Left.resetPosition();
      Right.resetPosition();
    }
    lastStick3 = stickTest3;
  }
  std::cout << "DTrain.stop(brake);" << std::endl;
  return 1;
}
int stick1gyro(){
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  int newHeading = 0;
  stopTimer = 0;
  while(floor(Controller.Axis1.position()/10)*10 != 0){
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    DTrain.turn(right, stickTest1, velocityUnits::pct);
    if(lastStick1 == 0){
      void;
    }
    else if(lastStick1 != stickTest1){
      std::cout << "DTrain.turn(right, " << lastStick1 << ", velocityUnits::pct);" << std::endl;
      std::cout << "waitUntil(" << Inertial.heading()-0.5 << " < Inertial.heading() && Inertial.heading() < " << Inertial.heading() << ");" << std::endl;
    }
    lastStick1 = stickTest1;
  }
  std::cout << "Left.spin(fwd, " << lastStick1 << ", pct);" << std::endl;
  std::cout << "waitUntil(" << Inertial.heading()-0.5 << " < Inertial.heading() && Inertial.heading() < " << Inertial.heading() << ");" << std::endl;
  std::cout << "DTrain.stop(brake);" << std::endl;
  return 1;
}
int stick1rev(){
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  stopTimer = 0;
  double DTrainrev;
  Left.resetPosition();
  Right.resetPosition();
  while(floor(Controller.Axis1.position()/10)*10 != 0){
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    Left.spin(fwd, stickTest1, pct);
    Right.spin(reverse, stickTest1, pct);
    if(stickTest1 == 0){
      void;
    }
    else if(lastStick1 != stickTest1){
      DTrainrev = (Left.position(rev)+Right.position(rev))/2; 
      std::cout.precision(3); //the next line has to be very long i am so sorry
      std::cout << "speedForGroup(Left, fwd, " << DTrainrev << ", " << lastStick1 << ", false);" << std::endl;
      std::cout << "speedForGroup(Right, reverse, " << DTrainrev << ", " << lastStick1 << ");" << std::endl;
      Left.resetPosition();
      Right.resetPosition();
    }
    lastStick1 = stickTest1;
  }
  DTrainrev = (Left.position(rev)+Right.position(rev))/2;
  std::cout << "speedForGroup(Left, fwd, " << DTrainrev << ", " << lastStick1 << ", false);" << std::endl;
  std::cout << "speedForGroup(Right, reverse, " << DTrainrev << ", " << lastStick1 << ");" << std::endl;
  std::cout << "DTrain.stop(brake);" << std::endl;
  return 1;
}

void driver(){
  wait(2, sec); //calibration
  std::cout << "wait(2, sec);" << std::endl;
  while(1){
    stickTest3 = floor(Controller.Axis3.position()/10)*10;
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    if(stickTest3 != 0){
      stick3rev();
    }
    else if(stickTest1 != 0){
      stick1gyro();
    }
    else{
      wait(1, msec);
      stopTimer += 1;
      lastStick3 = 0;
      DTrain.stop(brake);
    }
  }
}

void auton(){
wait(2, sec);
wait(0, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 5.4233, 100);
speedForGroup(GDTrain, fwd, 0.082222, 80);
speedForGroup(GDTrain, fwd, 0.082778, 50);
DTrain.stop(brake);
wait(391, msec);
DTrain.turn(right, 70, velocityUnits::pct);
waitUntil(358.62 < Inertial.heading() && Inertial.heading() < 359.12);
DTrain.turn(right, 100, velocityUnits::pct);
waitUntil(108.98 < Inertial.heading() && Inertial.heading() < 109.48);
Left.spin(fwd, 90, pct);
waitUntil(124.11 < Inertial.heading() && Inertial.heading() < 124.61);
DTrain.stop(brake);
waitUntil(Inertial.heading() < 161.91 && Inertial.heading() < 162.41);
DTrain.turn(right, 50, velocityUnits::pct);
waitUntil(161.91 < Inertial.heading() && Inertial.heading() < 162.41);
DTrain.turn(right, 80, velocityUnits::pct);
waitUntil(161.91 < Inertial.heading() && Inertial.heading() < 162.41);
DTrain.turn(right, 90, velocityUnits::pct);
waitUntil(161.92 < Inertial.heading() && Inertial.heading() < 162.42);
DTrain.turn(right, 100, velocityUnits::pct);
waitUntil(161.93 < Inertial.heading() && Inertial.heading() < 162.43);
DTrain.turn(right, 90, velocityUnits::pct);
waitUntil(162.6 < Inertial.heading() && Inertial.heading() < 163.1);
Left.spin(fwd, 50, pct);
waitUntil(164.29 < Inertial.heading() && Inertial.heading() < 164.79);
DTrain.stop(brake);
wait(236, msec);
DTrain.turn(right, 50, velocityUnits::pct);
waitUntil(178.33 < Inertial.heading() && Inertial.heading() < 178.83);
DTrain.turn(right, 10, velocityUnits::pct);
waitUntil(178.31 < Inertial.heading() && Inertial.heading() < 178.81);
DTrain.turn(right, 40, velocityUnits::pct);
waitUntil(178.28 < Inertial.heading() && Inertial.heading() < 178.78);
Left.spin(fwd, 30, pct);
waitUntil(178.28 < Inertial.heading() && Inertial.heading() < 178.78);
DTrain.stop(brake);
wait(192, msec);
DTrain.turn(right, 30, velocityUnits::pct);
waitUntil(178.24 < Inertial.heading() && Inertial.heading() < 178.74);
DTrain.turn(right, 40, velocityUnits::pct);
waitUntil(178.24 < Inertial.heading() && Inertial.heading() < 178.74);
DTrain.turn(right, 70, velocityUnits::pct);
waitUntil(178.24 < Inertial.heading() && Inertial.heading() < 178.74);
DTrain.turn(right, 80, velocityUnits::pct);
waitUntil(178.24 < Inertial.heading() && Inertial.heading() < 178.74);
Left.spin(fwd, 70, pct);
waitUntil(178.23 < Inertial.heading() && Inertial.heading() < 178.73);
DTrain.stop(brake);
wait(238, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0, 10);
speedForGroup(GDTrain, fwd, 0.0027778, 50);
speedForGroup(GDTrain, fwd, 0.0016667, 80);
speedForGroup(GDTrain, fwd, 5.3772, 100);
speedForGroup(GDTrain, fwd, 0.083333, 90);
DTrain.stop(brake);
wait(353, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, -0.0011111, 10);
speedForGroup(GDTrain, fwd, -0.0016667, 40);
speedForGroup(GDTrain, fwd, 0.00055556, 70);
speedForGroup(GDTrain, fwd, 0.03, 80);
speedForGroup(GDTrain, fwd, 0.019444, 90);
speedForGroup(GDTrain, fwd, 0.020556, 70);
DTrain.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  Inertial.calibrate();
  while(1){
    double stickTest = floor(Controller.Axis3.position()/10)*10;
    Brain.Screen.printAt(20, 20, "Stick Value: %f", stickTest);
    Brain.Screen.printAt(20, 40, "Gyro: %f", Inertial.heading());
  }
}
