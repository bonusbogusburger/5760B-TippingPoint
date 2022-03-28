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
  GDTrain.resetPosition();
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
      //std::cout << "Accelerated from " << lastStick3 << " to " << stickTest3 << " (GPS X: " << GPS.xPosition(inches) << " Y: " << GPS.yPosition(inches) << ") " << std::endl;
      double DTrainrev = (Left.position(rev)+Right.position(rev))/2; 
      //std::cout << "speedForGroup(Left, fwd, " << DTrainrev << ", " << lastStick3 << ", false);" << std::endl;
      //std::cout << "speedForGroup(Right, fwd, " << DTrainrev << ", " << lastStick3 << ");" << std::endl;
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
void driver(){
  std::cout << "wait(2, sec)" << std::endl;
  double lastStick3 = 0;
  double lastStick1 = 0;
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
wait(1861, msec);
speedForGroup(GDTrain, fwd, 0, 0);
speedForGroup(GDTrain, fwd, 0.166, 20);
speedForGroup(GDTrain, fwd, 0.186, 30);
speedForGroup(GDTrain, fwd, 0.055, 40);
speedForGroup(GDTrain, fwd, 0.025, 30);
speedForGroup(GDTrain, fwd, 0.36, 40);
speedForGroup(GDTrain, fwd, 0.535, 50);
speedForGroup(GDTrain, fwd, 0.374, 60);
speedForGroup(GDTrain, fwd, 0.422, 70);
speedForGroup(GDTrain, fwd, 0.756, 80);
speedForGroup(GDTrain, fwd, 0.539, 90);
speedForGroup(GDTrain, fwd, 1.85, 100);
speedForGroup(GDTrain, fwd, 0.167, 90);
DTrain.stop(brake);
wait(860, msec);
speedForGroup(GDTrain, fwd, 0, 50);
speedForGroup(GDTrain, fwd, -0.678, -40);
speedForGroup(GDTrain, fwd, -0.676, -50);
speedForGroup(GDTrain, fwd, -0.0256, -70);
speedForGroup(GDTrain, fwd, -0.0328, -90);
speedForGroup(GDTrain, fwd, -1.66, -100);
speedForGroup(GDTrain, fwd, -0.0861, -90);
speedForGroup(GDTrain, fwd, -0.085, -80);
speedForGroup(GDTrain, fwd, -0.0428, -70);
speedForGroup(GDTrain, fwd, -0.161, -60);
speedForGroup(GDTrain, fwd, -0.202, -50);
speedForGroup(GDTrain, fwd, -0.26, -40);
speedForGroup(GDTrain, fwd, -0.0539, -30);
speedForGroup(GDTrain, fwd, -0.312, -20);
speedForGroup(GDTrain, fwd, -0.00944, -10);
speedForGroup(GDTrain, fwd, -0.0167, -20);
speedForGroup(GDTrain, fwd, -0.0478, -10);
speedForGroup(GDTrain, fwd, -0.0167, -20);
DTrain.stop(brake);
wait(639, msec);
speedForGroup(GDTrain, fwd, -0.106, -10);
DTrain.stop(brake);
wait(1095, msec);
speedForGroup(GDTrain, fwd, 0, -20);
speedForGroup(GDTrain, fwd, -0.085, -10);
DTrain.stop(brake);
wait(450, msec);
speedForGroup(GDTrain, fwd, 0, -20);
speedForGroup(GDTrain, fwd, 0, -10);
speedForGroup(GDTrain, fwd, 0, -20);
speedForGroup(GDTrain, fwd, -0.0139, -30);
DTrain.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    double stickTest = floor(Controller.Axis3.position()/10)*10;
    Brain.Screen.printAt(20, 20, "Stick Value: %f", stickTest);
  }
}
