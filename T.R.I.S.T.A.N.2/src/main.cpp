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
    if(lastStick3 != stickTest3){
      std::cout.precision(3); //the next line has to be very long i am so sorry
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
  Inertial.resetHeading();
  int newHeading = 0;
  stopTimer = 0;
  while(floor(Controller.Axis1.position()/10)*10 != 0){
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    Left.spin(fwd, stickTest1, pct);
    Right.spin(reverse, stickTest1, pct);
    if(stickTest1 == 0){
      void;
    }
    if(lastStick1 != stickTest1){
      std::cout.precision(3); //the next line has to be very long i am so sorry
      std::cout << "Left.spin(fwd, " << lastStick1 << ", pct);" << std::endl;
      std::cout << "Right.spin(reverse, " << lastStick1 << ", pct);" << std::endl;
      std::cout << "waitUntil(Inertial.heading() == " << Inertial.heading() << ");" << std::endl;
      Inertial.resetHeading();
    }
    lastStick1 = stickTest1;
  }
  std::cout << "Left.spin(fwd, " << lastStick1 << ", pct);" << std::endl;
  std::cout << "Right.spin(reverse, " << lastStick1 << ", pct);" << std::endl;
  std::cout << "waitUntil(fabs(Inertial.heading()) >= " << fabs(Inertial.heading()) << ");" << std::endl;
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
    if(lastStick1 != stickTest1){
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
  double lastStick3 = 0;
  double lastStick1 = 0;
  wait(2, sec); //calibration
  std::cout << "wait(2, sec);" << std::endl;
  while(1){
    stickTest3 = floor(Controller.Axis3.position()/10)*10;
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    if(stickTest3 != 0){
      stick3rev();
    }
    else if(stickTest1 != 0){
      stick1rev();
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
speedForGroup(GDTrain, fwd, 5.3, 100);
DTrain.stop(brake);
wait(121, msec);
speedForGroup(Left, fwd, 0, 0, false);
speedForGroup(Right, reverse, 0, 0);
speedForGroup(Left, fwd, 0.0133, -40, false);
speedForGroup(Right, reverse, 0.0133, -40);
speedForGroup(Left, fwd, 0.00556, -70, false);
speedForGroup(Right, reverse, 0.00556, -70);
speedForGroup(Left, fwd, -0.0272, -100, false);
speedForGroup(Right, reverse, -0.0272, -100);
speedForGroup(Left, fwd, -0.00111, -10, false);
speedForGroup(Right, reverse, -0.00111, -10);
DTrain.stop(brake);
wait(114, msec);
speedForGroup(GDTrain, fwd, 0, 40);
speedForGroup(GDTrain, fwd, 0.00111, 20);
speedForGroup(GDTrain, fwd, 0.000556, 40);
speedForGroup(GDTrain, fwd, 0.00778, 50);
speedForGroup(GDTrain, fwd, 2.42, 100);
speedForGroup(GDTrain, fwd, 0.0778, 90);
DTrain.stop(brake);
wait(194, msec);
speedForGroup(Left, fwd, 0, -10, false);
speedForGroup(Right, reverse, 0, -10);
speedForGroup(Left, fwd, 0.00778, 20, false);
speedForGroup(Right, reverse, 0.00778, 20);
speedForGroup(Left, fwd, 0.00556, 30, false);
speedForGroup(Right, reverse, 0.00556, 30);
speedForGroup(Left, fwd, 0, 80, false);
speedForGroup(Right, reverse, 0, 80);
speedForGroup(Left, fwd, -0.0483, 100, false);
speedForGroup(Right, reverse, -0.0483, 100);
DTrain.stop(brake);
wait(64, msec);
speedForGroup(GDTrain, fwd, 0, 60);
speedForGroup(GDTrain, fwd, -0.005, 10);
speedForGroup(GDTrain, fwd, 0.00111, 20);
speedForGroup(GDTrain, fwd, 0, 60);
speedForGroup(GDTrain, fwd, 1.75, 100);
DTrain.stop(brake);
wait(371, msec);
speedForGroup(Left, fwd, 0, 100, false);
speedForGroup(Right, reverse, 0, 100);
speedForGroup(Left, fwd, -0.00167, 40, false);
speedForGroup(Right, reverse, -0.00167, 40);
speedForGroup(Left, fwd, -0.00167, 70, false);
speedForGroup(Right, reverse, -0.00167, 70);
speedForGroup(Left, fwd, -0.000556, 90, false);
speedForGroup(Right, reverse, -0.000556, 90);
speedForGroup(Left, fwd, 0.00667, 100, false);
speedForGroup(Right, reverse, 0.00667, 100);
speedForGroup(Left, fwd, 0.00556, 50, false);
speedForGroup(Right, reverse, 0.00556, 50);
DTrain.stop(brake);
wait(149, msec);
speedForGroup(Left, fwd, 0, 50, false);
speedForGroup(Right, reverse, 0, 50);
speedForGroup(Left, fwd, 0, 20, false);
speedForGroup(Right, reverse, 0, 20);
speedForGroup(Left, fwd, 0, 60, false);
speedForGroup(Right, reverse, 0, 60);
speedForGroup(Left, fwd, 0, 90, false);
speedForGroup(Right, reverse, 0, 90);
speedForGroup(Left, fwd, -0.00389, 100, false);
speedForGroup(Right, reverse, -0.00389, 100);
DTrain.stop(brake);
wait(212, msec);
speedForGroup(Left, fwd, 0, 100, false);
speedForGroup(Right, reverse, 0, 100);
speedForGroup(Left, fwd, 0, 10, false);
speedForGroup(Right, reverse, 0, 10);
speedForGroup(Left, fwd, 0, 40, false);
speedForGroup(Right, reverse, 0, 40);
speedForGroup(Left, fwd, 0.000556, 60, false);
speedForGroup(Right, reverse, 0.000556, 60);
speedForGroup(Left, fwd, 0.00111, 70, false);
speedForGroup(Right, reverse, 0.00111, 70);
DTrain.stop(brake);
wait(220, msec);
speedForGroup(GDTrain, fwd, 0, 50);
speedForGroup(GDTrain, fwd, 0, -10);
speedForGroup(GDTrain, fwd, -0.000556, -20);
speedForGroup(GDTrain, fwd, -0.628, -30);
speedForGroup(GDTrain, fwd, -0.0122, -20);
speedForGroup(GDTrain, fwd, -0.286, -30);
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
  }
}
