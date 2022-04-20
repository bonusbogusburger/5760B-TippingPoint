 // ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// MotorGroup3          motor_group   3, 5            
// ---- END VEXCODE CONFIGURED DEVICES ----
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
#include <visionsensor.h>

using namespace vex;

double shift = 1;
double toDegrees = 180/M_PI;

//define competition functions/controllers
competition Competition;
controller Controller1;
controller Controller2;

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
motor IL(PORT21, ratio18_1,true);
motor Hook(PORT16);
motor Motor(PORT17);

//define sensors
gps GPS(PORT9);
distance DistanceL(PORT18);
distance DistanceR(PORT19);
inertial Inertia(PORT2);
vision Vision(PORT17);
triport ThreeWirePort = vex::triport( vex::PORT22 );
vex::limit HookLimit = vex::limit(ThreeWirePort.A);

//Switch between 2 different controllers for driver control
controller CurDrive = Controller1;
int vCurDrive = 1;

//gear shift task
int gearShift(){
  CurDrive.Screen.setCursor(3, 1);
  CurDrive.Screen.print("MAX DUMPY");
  while(1){
    if(CurDrive.ButtonL2.pressing()){
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

//Vision Sensor Testing
void LineUpY(){
Vision1.setBrightness(50);
Vision1.setSignature(YGoal);
//camera image is 316 pixels wide
int screen_middle_x = 316 / 2;
bool linedup = false;
  while(not linedup) {
    Vision1.takeSnapshot(YGoal);
    if(Vision1.objectCount > 0) {
      if(Vision1.largestObject.centerX < screen_middle_x - 5) {
        //On the left, turning left
        Left.spin(reverse, 5, pct);  
        Right.spin(reverse, 5, pct);
        wait(0.1, sec);
      } 
      else if (Vision1.largestObject.centerX > screen_middle_x + 5) {
        //On the right, turning right
        Left.spin(forward, 5, pct);  
        Right.spin(forward, 5, pct); 
        wait(0.1, sec);
      } 
      else {
        //Done lining up
        linedup = true;
        Left.stop(coast);
        Right.stop(coast);
                
      }
    } 
    else {
      Left.stop(coast);
      Right.stop(coast);      
            
    }
  }
}

//PID Testing
void PIDturnToValue(){
  while((Inertia.heading( rotationUnits::deg )>358) == 0)  {//turned right correct left
   if( 60 > Inertia.heading( rotationUnits::deg )){
    Left.spin(reverse, 5, pct);  
    Right.spin(reverse, 5, pct);
    waitUntil(Inertia.heading( rotationUnits::deg ) > 300);
  }
  else if (300 < Inertia.heading( rotationUnits::deg )){//turned left correct right
    Left.spin(forward, 5, pct);  
    Right.spin(forward, 5, pct);
    waitUntil(60 > Inertia.heading( rotationUnits::deg ));
  }
  }
}

void PIDstraight(){
  int error;
  int speedL; 
  int speedR;
  double desiredValue = 359;
  int currentValue = Inertia.heading( rotationUnits::deg );
  int kP = 50;
  error = currentValue - desiredValue;
  speedL = (error/kP)*-1;
  speedR = (error/kP);
  while(1){
  Left.spin(forward, 10 + speedL, pct);  
  Right.spin(reverse, 10 + speedR, pct); 
  this_thread::sleep_for(30);
  }
}

//Inertial Gyro Testing
void TurnLeft(){
  Left.spin(reverse, 90, pct);  
  Right.spin(reverse, 90, pct);
  wait(0.5, sec); 
  Left.stop();
  Right.stop();
  wait(0.3, sec);
  while( Inertia.heading( rotationUnits::deg ) <= 269 ) {
    Left.spin(forward, 7, pct);  
    Right.spin(forward, 7, pct);  
    this_thread::sleep_for(10);
  }
  Left.stop(brake);
  Right.stop(brake);
  Inertia.calibrate();
}

void TurnRight(){
  Left.spin(forward, 50, pct);  
  Right.spin(forward, 50, pct);
  waitUntil( 20 > Inertia.heading( rotationUnits::deg ));
  Left.spin(forward, 50, pct);  
  Right.spin(forward, 50, pct);
  waitUntil(40 < Inertia.heading( rotationUnits::deg ));
  Left.stop();
  Right.stop();
  wait(0.4, sec);
  while( Inertia.heading( rotationUnits::deg ) >= 91 ) {
    Left.spin(reverse, 7, pct);  
    Right.spin(reverse, 7, pct);  
    this_thread::sleep_for(10);
  }
  Left.stop(brake);
  Right.stop(brake);
  Inertia.calibrate();
}

//spinFor but with speed to make autonomous less of a pain
void speedFor(motor Motor, directionType direct, double rotations, double speed, bool waitfor=true){
  Motor.setVelocity(speed, pct);
  Motor.spinFor(direct, rotations, rev, waitfor);
}

//same thing but for motor groups
void speedForGroup(motor_group MotorGroup, directionType direct, double rotations, double speed, bool waitfor=true){
  MotorGroup.setVelocity(speed, pct);
  MotorGroup.spinFor(direct, rotations, rev, waitfor);
}


int dropHook(){ //task to drop the hook
  while(HookLimit.pressing() != 1){
    Hook.spin(reverse, 100, pct);
  }
  Hook.stop(hold);
  return 0;
}

//Super Cool GPS Functions
//prototype 
void moveTo(double xFinal, double yFinal, double speed){ //robot calculates angle it needs to turn in order to drive to a desired location
  double deltaX = xFinal - GPS.xPosition(mm);
  double deltaY = yFinal - GPS.yPosition(mm);
  double turnAngle = atan(deltaY/deltaX)*toDegrees; //trigonometry!
  double driveDistance = sqrt((deltaX*deltaX) + (deltaY*deltaY));
  if(deltaY < 0){
    turnAngle += 180;
  }
  while(GPS.heading() != turnAngle){
    Left.spin(fwd, 75, pct);
    Right.spin(fwd, 75, pct);
  }
  DTrain.driveFor(driveDistance, mm); //no clue if this'll work
}

//drive to GPS x or y coordinates
double xp = 0;
double yp = 0;
double coordspeed = 0;
directionType coorddirect = fwd;
void driveToX(double x, directionType direct, double speed){
  if(x - GPS.xPosition(inches) > 0){
    while(GPS.xPosition(inches) <= x){
      DTrain.drive(direct, speed, velocityUnits::pct);
    }
  }
  else if(x - GPS.xPosition(inches) < 0){
    while(GPS.xPosition(inches) >= x){
      DTrain.drive(direct, speed, velocityUnits::pct);
    }
  }
}
void driveToY(double y, directionType direct, double speed){
  if(y - GPS.yPosition(inches) > 0){
    while(GPS.xPosition(inches) <= y){
      DTrain.drive(direct, speed, velocityUnits::pct);
    }
  }
  else if(y - GPS.yPosition(inches) < 0){
    while(GPS.yPosition(inches) >= y){
      DTrain.drive(direct, speed, velocityUnits::pct);
    }
  }
}

void dTCoordParams(double xpos, double ypos, directionType directio, double speed){ //workaround for tasks not having parameters
  xp = xpos;
  yp = ypos;
  coorddirect = directio;
  coordspeed = speed;
}
int driveToXTask(){ //these are in case we want to make these non-blocking
  if(xp - GPS.xPosition(inches) > 0){
    while(GPS.xPosition(inches) <= xp){
      DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
    }
  }
  else if(xp - GPS.xPosition(inches) < 0){
    while(GPS.xPosition(inches) >= xp){
      DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
    }
  }
  return xp;
}
int driveToYTask(){
  if(yp - GPS.xPosition(inches) > 0){
    while(GPS.xPosition(inches) <= yp){
      DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
    }
  }
  else if(yp - GPS.xPosition(inches) < 0){
    while(GPS.xPosition(inches) >= yp){
      DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
    }
  }
  return yp;
}

//autonomous routines
double timee = 0; 
double timeTo = 0;
int timeLimit(){ //task that sets a time limit
  timee = 0;
  wait(timeTo, sec);
  timee = 1;
  return timee;
}

int dropLift(){ //non-blocking task that drops the lift from starting position
  RRelease.open();
  IL.spinFor(fwd, 7, rev);
  RRelease.close();
  return 1;
}
void rightTime(){ //Be wary of rule SG4
  //begin autonomous
  task yeah(dropHook);
  wait(0.4, sec);
  RightClamp.close();

  //drive into goal
  DTrain.drive(reverse, 100, velocityUnits::pct);
  wait(1, sec);
  RightClamp.open();
  wait(0.035, sec);
  DTrain.stop(brake);
  Hook.spinFor(fwd, 0.05, rev);
  wait(0.15, sec);

  //move with goal
  DTrain.drive(fwd, 100, velocityUnits::pct);
  wait(0.5, sec);
  DTrain.stop(hold);

  //turn and prep for middle goal w/ left claw
  speedForGroup(Right, fwd, 0.35, 50, false);
  speedForGroup(Left, reverse, 0.35, 50);
  DTrain.stop(brake);
  wait(0.1, sec);

  //drive to grab goal
  wait(0.05, sec);
  LeftClamp.close();
  DTrain.drive(reverse, 80, velocityUnits::pct);
  wait(0.7, sec);
  LeftClamp.open();
  DTrain.drive(reverse, 80, velocityUnits::pct);
  wait(0.2, sec);
  Hook.spinFor(fwd, 0.5, rev, false);
  DTrain.stop(brake);

  //back up with middle goal
  DTrain.drive(fwd, 80, velocityUnits::pct);
  wait(1.35, sec);
  DTrain.stop(brake);
  wait(0.5, sec);

  //turn to alliance goal
  task yah(dropLift);
  speedForGroup(Right, fwd, 0.285, 50, false);
  speedForGroup(Left, reverse, 0.285, 50);
  DTrain.stop(brake);
  DTrain.drive(reverse, 25, velocityUnits::pct);
  wait(0.3, sec);
  DTrain.stop(brake);
  wait(1.5, sec);

  //grab alliance goal
  DTrain.drive(fwd, 50, velocityUnits::pct);
  wait(1.3, sec);
  IL.setVelocity(50, pct);
  IL.spinFor(reverse, 2.25, rev, false);
  DTrain.drive(reverse, 30, velocityUnits::pct);
  wait(0.125, sec);
  DTrain.stop(brake);
}

void leftTime(){
  //begin autonomous
  task yeah(dropHook);
  wait(0.4, sec);
  RightClamp.close();

  //drive into goal
  DTrain.drive(reverse, 100, velocityUnits::pct);
  wait(1, sec);
  RightClamp.open();
  wait(0.035, sec);
  DTrain.stop(brake);
  Hook.spinFor(fwd, 0.05, rev);
  wait(0.15, sec);

  //move with goal
  DTrain.drive(fwd, 100, velocityUnits::pct);
  wait(1.5, sec);
  DTrain.stop(hold);
}

void skills(){ 
  wait(1.5, sec); //calibrate GPS and inertial
  //begin autonomous
  task yeah(dropHook);
  wait(0.4, sec);
  RightClamp.close();

  //drive into goal
  DTrain.drive(reverse, 100, velocityUnits::pct);
  wait(1, sec);
  RightClamp.open();
  wait(0.035, sec);
  DTrain.stop(brake);
  Hook.spinFor(fwd, 0.05, rev);
  wait(0.15, sec);

  //move with goal
  DTrain.drive(fwd, 100, velocityUnits::pct);
  wait(0.5, sec);
  DTrain.stop(hold);

  //turn and prep for middle goal w/ left claw
  speedForGroup(Right, fwd, 0.35, 50, false);
  speedForGroup(Left, reverse, 0.35, 50);
  DTrain.stop(brake);
  wait(0.1, sec);

  //drive to grab goal
  wait(0.05, sec);
  LeftClamp.close();
  DTrain.drive(reverse, 80, velocityUnits::pct);
  wait(0.7, sec);
  LeftClamp.open();
  DTrain.drive(reverse, 80, velocityUnits::pct);
  wait(0.2, sec);
  Hook.spinFor(fwd, 0.5, rev, false);
  DTrain.stop(brake);

  //back up with middle goal
  DTrain.drive(fwd, 80, velocityUnits::pct);
  wait(1.35, sec);
  DTrain.stop(brake);
  wait(0.5, sec);

  //turn to alliance goal
  task yah(dropLift);
  speedForGroup(Right, fwd, 0.285, 50, false);
  speedForGroup(Left, reverse, 0.285, 50);
  DTrain.stop(brake);
  DTrain.drive(reverse, 25, velocityUnits::pct);
  wait(0.3, sec);
  DTrain.stop(brake);
  wait(1.5, sec);

  //grab alliance goal, turn to other side
  DTrain.drive(fwd, 50, velocityUnits::pct);
  wait(1.3, sec);
  IL.setVelocity(50, pct);
  IL.spinFor(reverse, 2.25, rev, false);
  DTrain.drive(reverse, 30, velocityUnits::pct);
  wait(0.125, sec);
  DTrain.stop(brake);
  Right.spin(fwd, 25, pct);
  Left.spin(reverse, 25, pct);
  waitUntil(Inertia.heading(degrees) < 182);
  DTrain.stop(hold);
  wait(1, sec);

  //Pick up, drop rings, drop goal, back up
  DTrain.drive(fwd, 30, velocityUnits::pct);
  IL.spin(fwd, 100, pct);
  waitUntil(GPS.xPosition(inches) > 49);
  DTrain.stop(brake);
  wait(0.5, sec);
  /*RRelease.open();
  IL.spin(reverse, 100, pct); //this releases tension on the ratchet
  wait(1, sec);
  IL.spin(fwd, 100, pct);
  wait(3, sec);
  RRelease.close();
  IL.stop(hold);
  DTrain.drive(reverse, 30, velocityUnits::pct);
  wait(0.8, sec);
  DTrain.stop(brake);
  wait(0.5, sec);*/

  //Turn and get red goal!!!
  /*Left.spin(reverse, 25, pct);
  Right.spin(fwd, 25, pct);
  waitUntil(Inertia.heading(deg) < 90);
  DTrain.stop(brake);*/
}

void auton(){ //plan is to use a limit switch/bumper/other sensor to select an autonomous routine before a match
  RRelease.open();
  RightClamp.open();
  LeftClamp.close();
  leftTime();
}

void driver(){
  //intake = false;
  task jank(gearShift);
  //task toggle(intakeToggle);
  RRelease.open();
  RightClamp.open();
  LeftClamp.close();
  while(1){
    //drivetrain brake
     if(CurDrive.ButtonLeft.pressing()){
      DTrain.stop(hold);
    }
    else{
    //drivetrain (tank)
    Left.spin(fwd, CurDrive.Axis3.position()*shift, pct);
    Right.spin(fwd, CurDrive.Axis2.position()*shift, pct);
    }

    if(CurDrive.ButtonL1.pressing()){
      IL.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonR1.pressing()){
      IL.spin(reverse, 65*shift, pct);
    }
    else{
      IL.stop(coast);
    }

    if(CurDrive.ButtonR2.pressing()){
      RRelease.open();
    }
    else{
      RRelease.close();
    }

    if(HookLimit.pressing()==0){
      if(CurDrive.ButtonB.pressing()){
      Hook.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonDown.pressing()){
      Hook.spin(reverse, 100, pct);
    }
    else{
      Hook.stop(hold);
    }
    }

    if(HookLimit.pressing()==1){
      if(CurDrive.ButtonB.pressing()){
      Hook.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonDown.pressing()){
      Hook.stop(hold);
    }
    else{
      Hook.stop(hold);
    }
    }

    if(CurDrive.ButtonRight.pressing()){
      LeftClamp.close();
    }
    else{
      LeftClamp.open();
    }

    if(CurDrive.ButtonY.pressing()){
      RightClamp.close();
    }
    else{
      RightClamp.open();
    }

    if(CurDrive.ButtonUp.pressing()){ //just for easy motor testing
      Motor.spin(fwd, 50, pct);
    }
    else{
      Motor.stop(coast);
    }
  }
}

int main(){
  // Initializing Robot Configuration. DO NOT REMOVE! (ok)
  vexcodeInit();
  IL.setVelocity(100, pct);
  Hook.setVelocity(100, pct); 
  Competition.drivercontrol(driver);
  Competition.autonomous(auton);
  while(1){
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(monoM);
    Brain.Screen.printAt(20, 20, "Inertial: %f deg", Inertia.heading());
    Brain.Screen.printAt(20, 40, "GPS Gyro: %f deg", GPS.heading(degrees));
    Brain.Screen.printAt(20, 60, "Left1 Temp: %f ℃", Left1.temperature(celsius));
    Brain.Screen.printAt(20, 80, "Left2 Temp: %f ℃", Left2.temperature(celsius));
    Brain.Screen.printAt(20, 100, "Left3 Temp: %f ℃", Left3.temperature(celsius));
    Brain.Screen.printAt(20, 120, "Right1 Temp: %f ℃", Right1.temperature(celsius));
    Brain.Screen.printAt(20, 140, "Right2 Temp: %f ℃", Right2.temperature(celsius));
    Brain.Screen.printAt(20, 160, "Right3 Temp: %f ℃", Right3.temperature(celsius));
    Brain.Screen.printAt(20, 180, "LDistance: %f", DistanceL.objectDistance(mm));
    Brain.Screen.setFont(propXXL);
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(280, 105, "5760B");
  }
}
