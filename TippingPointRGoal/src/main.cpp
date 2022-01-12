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
motor Left1(PORT18, ratio18_1);
motor Left2(PORT10 , ratio18_1, true);
motor Left3(PORT7, ratio18_1, true);
motor Right1(PORT14, ratio18_1);
motor Right2(PORT20, ratio18_1, true);
motor Right3(PORT13, ratio18_1, true);
motor_group Left(Left1, Left2, Left3);
motor_group Right(Right1, Right2, Right3);
drivetrain DTrain(Left, Right);
motor IL(PORT21, ratio18_1,true);
motor Hook(PORT16);
motor Motor(PORT17);

//define sensors
gps GPS(PORT7);
distance DistanceL(PORT9);
distance DistanceR(PORT19);
inertial Inertia(PORT11);
vision Vision(PORT17);
triport ThreeWirePort = vex::triport( vex::PORT22 );
vex::limit HookLimit = vex::limit(ThreeWirePort.A);

//Switch between 2 different controllers for driver control (refer to the jank function graveyard)
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


int dropHook(){ //task to drop the hook
  while(HookLimit.pressing() != 1){
    Hook.spin(fwd, 100, pct);
  }
  Hook.stop(hold);
  return 0;
}

//Super Cool GPS Functions
//prototype (i have no idea if this works)
void moveTo(double xFinal, double yFinal, double speed){ //robot calculates angle it needs to turn in otder to drive to a desired location
  double deltaX = xFinal - GPS.xPosition(mm);
  double deltaY = yFinal - GPS.yPosition(mm);
  double turnAngle = atan(deltaY/deltaX)*toDegrees; //trigonometry! my worst math unit!
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

//drive to GPS x or y coordinates (there's probably a much better way to go about this)
double x = 0;
double y = 0;
double coordspeed = 0;
directionType coorddirect = fwd;
void driveToX(double x, directionType direct, double speed){
  while(GPS.xPosition(mm) != x){
    DTrain.drive(direct, speed, velocityUnits::pct);
  }
}
void driveToY(double y, directionType direct, double speed){
  while(GPS.yPosition(mm) != y){
    DTrain.drive(direct, speed, velocityUnits::pct);
  }
}
void dTCoordParams(double xpos, double ypos, directionType directio, double speed){ //workaround for tasks not having parameters
  x = xpos;
  y = ypos;
  coorddirect = directio;
  coordspeed = speed;
}
int driveToXTask(){ //these are in case we want to make these non-blocking. this makes things less complicated trust me
  while(GPS.xPosition(mm) != x){
    DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
  }
  return 1;
}
int driveToYTask(){
  while(GPS.yPosition(mm) != y){
    DTrain.drive(coorddirect, coordspeed, velocityUnits::pct);
  }
  return 1;
}

//outdated, needs to be redone. completely. as a matter of fact it's BEING redone. NOW
void auton(){
  LeftClamp.close();
  task yeah(dropHook);
  driveToX(50, fwd, 100);
  LeftClamp.open();
  DTrain.stop(hold);
  driveToX(-850, reverse, 100);
  DTrain.stop(hold);

/*Left1.spin(forward, 100, pct);
wait(1, sec);
Left1.stop(coast);
Left2.spin(forward, 100, pct);
wait(1, sec);
Left2.stop(coast);
Left3.spin(forward, 100, pct);
wait(1, sec);
Left3.stop(coast);
Right1.spin(forward, 100, pct);
wait(1, sec);
Right1.stop(coast);
Right2.spin(forward, 100, pct);
wait(1, sec);
Right2.stop(coast);
Right3.spin(forward, 100, pct);
wait(1, sec);
Right3.stop(coast);*/
}

void driver(){
  task jank(gearShift);
  while(1){
    //drivetrain brake
     if(CurDrive.ButtonLeft.pressing()){
      DTrain.stop(hold);
    }
    else{
    //drivetrain (tank)
    Left.spin(fwd, CurDrive.Axis3.position()*shift, pct);
    Right.spin(reverse, CurDrive.Axis2.position()*shift, pct);
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
    /*if(CurDrive.ButtonB.pressing()){
      Hook.spin(fwd, 100, pct);
    }
    else if(CurDrive.ButtonDown.pressing()){
      Hook.spin(reverse, 100, pct);
    }
    else if(HookLimit.pressing()==0){
      Hook.stop(hold);
    }
    else{
      Hook.stop(hold);
    }*/

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
  GPS.calibrate();
  Inertia.calibrate();
  IL.resetPosition();
  RRelease.open();
  RightClamp.open();
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
