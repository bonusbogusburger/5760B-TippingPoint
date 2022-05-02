/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Robotics                                         */
/*    Created:      Wed Apr 20 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

//If there's a ~ in front of this program's name, it means it's ready for competition
//Skills code contains any unused functions that have been taken out of this code
#include "vex.h"
#include <iostream>
#include <string>
using namespace std;
using namespace vex;
#include "Fvis.h"

competition Competition;
controller Cont1(controllerType::primary);
controller Cont2(controllerType::partner);
brain Brain;

//Drive Motors (Drive Left/Right 1-3)
motor DL1(PORT15, ratio18_1, true);
motor DL2(PORT16, ratio18_1, false);
motor DL3(PORT19, ratio18_1, true);
motor DR1(PORT11, ratio18_1, false);
motor DR2(PORT12, ratio18_1, true);
motor DR3(PORT13, ratio18_1, false);
motor_group DL(DL1, DL2, DL3);
motor_group DR(DR1, DR2, DR3);
drivetrain DT(DL, DR);

//Clamp Lift + Intake Lift
motor CL(PORT8, ratio36_1, true);
motor IL(PORT1, ratio36_1);

//Expanders
triport Expander1(PORT6);

//Sensors
gps GPS(PORT9);
vex::distance Distance1(PORT5);
vex::distance Distance2(PORT20);
potV2 Pot1(Expander1.G);
vision Vision(PORT22);
limit ILimit(Brain.ThreeWirePort.H);
limit CLimit(Brain.ThreeWirePort.C);
inertial Inertial1(PORT10);
inertial Inertial2(PORT6);

//Pneumatics
pneumatics Clamp(Brain.ThreeWirePort.B);
pneumatics ILift(Brain.ThreeWirePort.E);
pneumatics TransL(Brain.ThreeWirePort.D);
pneumatics TransR(Brain.ThreeWirePort.A);

//ooo shiny
led Green1(Expander1.A);
led Yellow1(Expander1.B);
led Red1(Expander1.C);

smartdrive SDT=smartdrive(DL, DR, Inertial1, 319.19, 406.4, 241.29999999999998, mm, 1.666666666666667);
bool autofunctions = true;

//motor spin function using voltage instead of pct or rpm that makes itself "easy" to use
void vspin(motor Motor, float value, string value_type = "pct", directionType direction = directionType::fwd){
  //determines motor spin direction using positive and negative votlages
  int dv;
  if(direction == directionType::fwd){
    dv = 1;
  }
  else if(direction == directionType::rev){
    dv = -1;
  }

  //figure out type of input value
  //math is based on a straightend curve value
  if(value_type == "pct"){
    Motor.spin(directionType::fwd, (((6*(2*value-120))/125)+6)*dv, voltageUnits::volt);
  }
  else if(value_type == "rpm"){
    Motor.spin(directionType::fwd, (((6*(value-120))/125)+6)*dv, voltageUnits::volt);
  }
  else if(value_type == "volt"){
    Motor.spin(directionType::fwd, value*dv, voltageUnits::volt);
  }
}

//motor group spin function using voltage instead of pct or rpm that makes itself "easy" to use
void gvspin(motor_group MotorGroup, float value, string value_type = "pct", directionType direction = directionType::fwd){
  //determines motor spin direction using positive and negative votlages
  int dv;
  if(direction == directionType::fwd){
    dv = 1;
  }
  else if(direction == directionType::rev){
    dv = -1;
  }

  //figure out type of input value
  //math is based on a straightend curve value
  if(value_type == "pct"){
    MotorGroup.spin(directionType::fwd, (((6*(2*value-120))/125)+6)*dv, voltageUnits::volt);
  }
  else if(value_type == "rpm"){
    MotorGroup.spin(directionType::fwd, (((6*(value-120))/125)+6)*dv, voltageUnits::volt);
  }
  else if(value_type == "volt"){
    MotorGroup.spin(directionType::fwd, value*dv, voltageUnits::volt);
  }
}

bool actuate;
void transToggle(){ //manually toggles transmission
  actuate = false;
  while(1){
    if(Cont1.ButtonRight.pressing() or Cont2.ButtonR2.pressing()){
      actuate = !actuate;
      wait(0.5, sec);
    }
    wait(5, msec);
  }
}

bool bactuate;
void backToggle(){ //toggles the solenoids on the back lift
  bactuate = false;
  while(1){
    if(Cont1.ButtonR1.pressing() or Cont2.ButtonR1.pressing()){
      bactuate = !bactuate;
      wait(0.5, sec);
    }
    wait(5, msec);
  }
}

bool cactuate;
void clampToggle(){ //toggles the front clamp
  cactuate = false;
  bool disdanc = false;
  while(1){
    if(Cont1.ButtonR2.pressing()){
      if(disdanc == true){
        cactuate = false;
        disdanc = false;
      }
      else if(disdanc == false){
        cactuate = !cactuate;
      }
      waitUntil(Cont1.ButtonR2.pressing() == false);
      wait(0.3, sec);
    }
    wait(5, msec);
  }
}

bool intaketoggle = false; //toggles the intake
void toggle(){
  while(1){
    if(Cont1.ButtonL2.pressing() or Cont2.ButtonL2.pressing()){
      intaketoggle = !intaketoggle;
      wait(0.3, sec);
    }
  }
}
thread toggler(toggle);

void autotoggle(){
  while(1){
    if(Cont2.ButtonLeft.pressing()or Cont1.ButtonLeft.pressing()){
    autofunctions = !autofunctions;
      wait(0.3, sec);
    }
  }
}
thread at(autotoggle);

void driver(){
  //thread weave(weaver);
  thread manual(transToggle);
  thread btoggle(backToggle);
  thread ctoggle(clampToggle);
  //weave.detach();
  manual.detach();
  btoggle.detach();
  ctoggle.detach();
  at.detach();
  while(1){
    if(fabs((floor(Cont1.Axis3.position()/10)*10)) > 0 or fabs((floor(Cont1.Axis2.position()/10)*10)) > 0){
      gvspin(DL, Cont1.Axis3.position());
      gvspin(DR, Cont1.Axis2.position());
    }
    else if(Cont1.ButtonUp.pressing()){
      gvspin(DL, 100);
      gvspin(DR, 100);
    }
    else{
      DT.stop(brake);
    }

    if(Cont1.ButtonB.pressing() and CLimit.pressing() == false){
      vspin(CL, 100);
    }
    else if(Cont1.ButtonDown.pressing() and ILimit.pressing() == false){
      vspin(CL, -100);
    }
    else{
      CL.stop(hold);
    }

    if(Cont1.ButtonL1.pressing() or Cont2.ButtonL1.pressing()){
      vspin(IL, -100);
      intaketoggle = false;
    }
    else if(intaketoggle){
      vspin(IL, 100);
    }
    else{
      IL.stop(hold);
    }

    if(cactuate == true){
      Clamp.close();
    }
    else if(cactuate == false){
      Clamp.open();
    }

    if(actuate == true){
      TransL.open();
      TransR.open();
    }
    else if(actuate == false){
      TransL.close();
      TransR.close();
    }

    if(bactuate == true){
      ILift.open();
    }
    else if(bactuate == false){
      ILift.close();
    }
  }
}

void dtvspin(double speed){ //gvspin but for the drivetrain as a whole
  gvspin(DL, speed);
  gvspin(DR, speed);
}

double desiredValue = 359;
void PIDstraight(double speed){ //uses a P system to keep the robot straight while driving
  int error;
  int speedL; 
  int speedR;
  int currentValue = Inertial1.heading( rotationUnits::deg );
  int kP = 50;
  error = currentValue - desiredValue;
  speedL = (error/kP)*-1;
  speedR = (error/kP);
  //DL.spin(fwd, 10 + speedL, pct); (old, keeping just in case)
  //DR.spin(reverse, 10 + speedR, pct); 
  gvspin(DL, speed + speedL);
  gvspin(DR, speed + speedR);
  this_thread::sleep_for(30);
}

void auton(){
  CL.spinFor(reverse, 1, rev, false);
  Clamp.open();
  SDT.setTurnConstant(1.15);
  SDT.turnToHeading(330, degrees, 65, velocityUnits::pct);
  desiredValue = Inertial1.heading();
  PIDstraight(100);
  waitUntil(Distance1.objectDistance(mm) < 25);
  Clamp.close();
  desiredValue = Inertial1.heading();
  wait(50, msec);
  TransL.open();
  TransR.open();
  CL.stop(hold);
  PIDstraight(-100);
  waitUntil(Distance2.objectDistance(mm) < 793);
  DT.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Clamp.open();
  TransL.close();
  TransR.close();
  Inertial1.calibrate();
  Inertial2.calibrate();
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);
  while(1){
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(monoM);
    Brain.Screen.printAt(20, 20, "DL1 Temp: %f", DL1.temperature(celsius));
    Brain.Screen.printAt(20, 40, "DL2 Temp: %f", DL2.temperature(celsius));
    Brain.Screen.printAt(20, 60, "DL3 Temp: %f", DL3.temperature(celsius));
    Brain.Screen.printAt(20, 80, "DR1 Temp: %f", DR1.temperature(celsius));
    Brain.Screen.printAt(20, 100, "DR2 Temp: %f", DR2.temperature(celsius));
    Brain.Screen.printAt(20, 120, "DR3 Temp: %f", DR3.temperature(celsius));
    Brain.Screen.printAt(20, 140, "SDT: %f deg", SDT.heading());
    Brain.Screen.setFont(propXXL);
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(280, 105, "5760B");
  }
}
