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

//modified version of speedForGroup from our competition programs that makes acceleration less jerky, some accuracy issues but pretty good overall
void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed, bool waitfor=true){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.resetPosition();
  MotorGroup.spin(fwd);
  waitUntil(fabs(MotorGroup.position(rev)) >= fabs(rotations));
  //MotorGroup.spinFor(direct, rotations, rev, waitfor);
}

//variables! some might be unnecessary but that's fine
double stickTest3;
double stickTest1;
int timer3 = 0; //only used for msec3 which doesn't work!
int timediff = 0; 
int stopTimer = 0; //counts the time in between driver functions (actually pretty accurate????!?!??)
double lastStick3;
double lastStick1;
int msec3(){ //timer task, can't get working right. doesn't get to be a part of the cool task club
  while(floor(Controller.Axis3.position()/10)*10 != 0){
    wait(1, msec);
    timer3 += 1;
    timediff += 1;
  }
  return 1;
}

//tasks! which are just normal functions right now because they won't run when they're tasks for some reason!
int stick3time(){ //scrapping in favour of the rotation method, keeping just in case
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
int stick3rev(){ //analog forward/backward with rotations
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
int stick1gyro(){ //analog turning with the inertial sensor's gyroscope
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
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
int stick1rev(){ //turning using rotations, doesn't work (yet?)
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
int dstick1gyro(){ //turning at a set speed using gyro
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  stopTimer = 0;
  int turnmult;
  while(Controller.Axis1.position() <= -50){
    DTrain.turn(left, 25, velocityUnits::pct);
    turnmult = -1;
  }
  while(Controller.Axis1.position() >= 50){
    DTrain.turn(right, 25, velocityUnits::pct);
    turnmult = 1;
  }
  std::cout << "DTrain.turn(right, 25*" << turnmult << ", velocityUnits::pct);" << std::endl;
  std::cout << "waitUntil(" << round(Inertial.heading())-1 << " < Inertial.heading() && Inertial.heading() < " << round(Inertial.heading()) << ");" << std::endl;
  std::cout << "DTrain.stop(brake);" << std::endl;
  return 1;
}
int dstick1gyro2(){ //turning at a set speed using gyro
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  double igyro = Inertial.heading();
  stopTimer = 0;
  int turnmult;
  while(Controller.Axis1.position() <= -50){
    DTrain.turn(left, 50, velocityUnits::pct);
    turnmult = -1;
  }
  while(Controller.Axis1.position() >= 50){
    DTrain.turn(right, 50, velocityUnits::pct);
    turnmult = 1;
  }
  if(Inertial.heading() > igyro){
    if(Inertial.heading() >= 355){
      std::cout << "DTrain.turn(right, 50, velocityUnits::pct);" << std::endl;
      std::cout << "waitUntil(Inertial.heading() >= 0" << std::endl;
    }
    std::cout << "DTrain.turn(right, 50, velocityUnits::pct);" << std::endl;
    std::cout << "waitUntil(Inertial.heading() >= " << Inertial.heading() << ");" << std::endl;
  }
  else if(Inertial.heading() > igyro){
    if(Inertial.heading() <= 5){
      std::cout << "DTrain.turn(left, 50, velocityUnits::pct);" << std::endl;
      std::cout << "waitUntil(Inertial.heading() <= 360);" << std::endl;
    }
    std::cout << "DTrain.turn(left, 50, velocityUnits::pct);" << std::endl;
    std::cout << "waitUntil(Inertial.heading() <= " << Inertial.heading() << ");" << std::endl;
  }
  return 1;
}
int dstick1rev(){ //turning at a set speed using rotations
  std::cout << "wait(" << stopTimer << ", msec);" << std::endl;
  stopTimer = 0;
  GDTrain.resetPosition();
  double avgrev;
  std::cout << "GDTrain.resetPosition();" << std::endl;
  int turnmult;
  while(Controller.Axis1.position() <= -50){
    DTrain.turn(left, 75, velocityUnits::pct);
    turnmult = -1;
  }
  while(Controller.Axis1.position() >= 50){
    DTrain.turn(right, 75, velocityUnits::pct);
    turnmult = 1;
  }
  avgrev = (fabs(Left.position(rev)) + fabs(Right.position(rev)))/2;
  std::cout << "DTrain.turn(right, 75*" << turnmult << ", velocityUnits::pct);" << std::endl;
  std::cout << "waitUntil(fabs(GDTrain.position(rev)) >= " << fabs(GDTrain.position(rev)) << ");" << std::endl;
  return 1;
}

void driver(){
  wait(2, sec); //sensor calibration
  std::cout << "wait(2, sec);" << std::endl;
  while(1){
    stickTest3 = floor(Controller.Axis3.position()/10)*10;
    stickTest1 = floor(Controller.Axis1.position()/10)*10;
    if(stickTest3 != 0){
      stick3rev();
    }
    else if(fabs(stickTest1) >= 50){
      dstick1gyro2();
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
speedForGroup(GDTrain, fwd, 5.88889, 100);
DTrain.stop(brake);
wait(312, msec);
wait(193, msec);
DTrain.turn(right, 50, velocityUnits::pct);
waitUntil(Inertial.heading() >= 159.64);
wait(282, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0, 10);
speedForGroup(GDTrain, fwd, 0.00111111, 60);
speedForGroup(GDTrain, fwd, 5.47722, 100);
speedForGroup(GDTrain, fwd, 0.118889, 80);
speedForGroup(GDTrain, fwd, 0.0827778, 70);
speedForGroup(GDTrain, fwd, 0.0738889, 60);
speedForGroup(GDTrain, fwd, 0.075, 50);
DTrain.stop(brake);
wait(1366, msec);
wait(115, msec);
wait(208, msec);
DTrain.turn(right, 50, velocityUnits::pct);
waitUntil(Inertial.heading() >= 0);
DTrain.turn(right, 50, velocityUnits::pct);
waitUntil(Inertial.heading() >= 356.047);
wait(213, msec);
wait(212, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0, -10);
speedForGroup(GDTrain, fwd, 0, -20);
speedForGroup(GDTrain, fwd, -0.000555556, -30);
speedForGroup(GDTrain, fwd, -0.0472222, -40);
speedForGroup(GDTrain, fwd, -0.0155556, -30);
DTrain.stop(brake);
wait(172, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0, -10);
speedForGroup(GDTrain, fwd, -0.0477778, -20);
DTrain.stop(brake);
wait(128, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0, -10);
speedForGroup(GDTrain, fwd, -0.0277778, -40);
speedForGroup(GDTrain, fwd, -0.0172222, -30);
DTrain.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  GPS.calibrate();
  Inertial.calibrate();
  while(1){
    double stickTest = floor(Controller.Axis3.position()/10)*10;
    Brain.Screen.printAt(20, 20, "Stick Value: %f", stickTest);
    Brain.Screen.printAt(20, 40, "Gyro: %f", Inertial.heading());
  }
}
