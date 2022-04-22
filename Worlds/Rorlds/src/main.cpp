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

#include "vex.h"
#include <iostream>
#include <string>
using namespace std;
using namespace vex;

competition Competition;
controller Cont1;
brain Brain;

//Ports are currently placeholders
//Drive Motors (Drive Left/Right 1-3)
motor DL1(PORT15, true);
motor DL2(PORT16, false);
motor DL3(PORT19, true);
motor DR1(PORT11, false);
motor DR2(PORT12, true);
motor DR3(PORT13, false);
motor_group DL(DL1, DL2, DL3);
motor_group DR(DR1, DR2, DR3);
drivetrain DT(DL, DR);

//Clamp Lift + Intake Lift
motor CL(PORT4, true);
motor IL(PORT1);

//Sensors
gps GPS(PORT9);
optical Optical(PORT6);
vex::distance Distance(PORT11); //damn you iostream
//potV2 Pot1();

//Expanders
triport Expander1(PORT6);

//Pneumatics
pneumatics Clamp(Brain.ThreeWirePort.B);
pneumatics ILift(Brain.ThreeWirePort.C);
pneumatics Trans(Brain.ThreeWirePort.D);

//ooo shiny
led Green1(Expander1.A);
led Green2(Expander1.B);
led Yellow1(Expander1.C);
led Yellow2(Expander1.D);
led Red1(Expander1.E);
led Red2(Expander1.F);

void oooShiny(){
  while(1){
    Green1.on();
    wait(50, msec);
    Green1.off();
    Green2.on();
    wait(50, msec);
    Green2.off();
    Yellow1.on();
    wait(50, msec);
    Yellow1.off();
    Yellow2.on();
    wait(50, msec);
    Yellow2.off();
    Red1.on();
    wait(50, msec);
    Red1.off();
    Red2.on();
    wait(50, msec);
    Red2.off();
    Red2.on();
    wait(50, msec);
    Red2.off();
    Red1.on();
    wait(50, msec);
    Red1.off();
    Yellow2.on();
    wait(50, msec);
    Yellow2.off();
    Yellow1.on();
    wait(50, msec);
    Yellow1.off();
    Green2.on();
    wait(50, msec);
    Green2.off();
    Green1.on();
    wait(50, msec);
    Green1.off();
  }
}

void motorspin(motor Motor, float value, string value_type = "pct", directionType direction = directionType::fwd){
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
void motorsping(motor_group MotorGroup, float value, string value_type = "pct", directionType direction = directionType::fwd){
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

bool automatic = true;
bool actuate = false;
float speedmod = 1;

void autoshifter(){
  float watt;
  while(1){
    watt = (DR.power()+DL.power())/2;
    if(watt < 0.5){
      speedmod = 1;
      actuate = false;
    }
    else if(watt<1.5){
      speedmod = 0.5;
      actuate = false;
    }
    else if(watt<4){
      speedmod = 1;
      actuate = true;
    }
    else if(watt<4){
      speedmod = 0.5;
      actuate = true;
    }
    wait(0.0000000000000000001,msec);
  }
}

void manualshifter(){
  int cycle = 0;
  while(1){
    if(Cont1.ButtonX.pressing() & (cycle == 0)){
      speedmod = 1;
      actuate = false;
      cycle = 1;
    }
    else if(Cont1.ButtonX.pressing() & (cycle == 1)){
      speedmod = 0.5;
      actuate = false;
      cycle = 2;
    }
    else if(Cont1.ButtonX.pressing() & (cycle == 2)){
      speedmod = 1;
      actuate = true;
      cycle = 3;
    }
    else if(Cont1.ButtonX.pressing() & (cycle == 0)){
      speedmod = 0.5;
      actuate = true;
      cycle = 0;
    }
    wait(0.25, sec);
  }
}


//thread parent, controls automatic and manual
void weaver(){
  //sense forchanges and record them
  int mode = 0;
  int cntmode = 0;
  vex::thread autoshift(autoshifter);
  autoshift.detach();
  vex::thread manual(manualshifter);
    manual.detach();
    manual.interrupt();
  while(1){
  if(Cont1.ButtonA.pressing() & (mode == 0)){
    mode = 1;
    automatic = false;
  }
  else if(Cont1.ButtonA.pressing() & (mode == 1)){
    mode = 0;
    automatic = true;
  }

  if((!automatic) & (cntmode == 1)){
    vex::thread manual(manualshifter);
    manual.detach();
    autoshift.interrupt();
    cntmode = 0;
  }
  else if (automatic & (cntmode == 0)){
  vex::thread autoshift(autoshifter);
  autoshift.detach();
  manual.interrupt();
  cntmode = 1;
  }
  wait(0.25, sec);
  }
}

void auton(){ 
  Clamp.open();
  wait(50, msec);
  motorsping(DL, 75);
  motorsping(DR, 75);
  waitUntil(Optical.brightness() < 2);
  Clamp.close();
  wait(50, msec);
  motorsping(DL, -50);
  motorsping(DR, -50);
  wait(1, sec);
  DT.stop(brake);
}

void driver(){
  vex::thread weave(weaver);
  weave.detach();
  while(1){
    if(fabs((floor(Cont1.Axis3.position()/10)*10)) > 0 or fabs((floor(Cont1.Axis2.position()/10)*10)) > 0){
      motorsping(DL, Cont1.Axis3.position()*speedmod);
      motorsping(DR, Cont1.Axis2.position()*speedmod);
    }
    else{
      DT.stop(brake);
    }

    if(Cont1.ButtonB.pressing()){
      motorspin(CL, 100);
    }
    else if(Cont1.ButtonDown.pressing()){
      motorspin(CL, -100);
    }
    else{
      CL.stop(hold);
    }

    if(Cont1.ButtonR1.pressing()){
      motorspin(IL, 100);
    }
    else if(Cont1.ButtonL1.pressing()){
      motorspin(IL, -100);
    }
    else{
      IL.stop(hold);
    }

    if(Cont1.ButtonY.pressing()){
      Clamp.close();
    }
    else{
      Clamp.open();
    }

    if(Cont1.ButtonRight.pressing()){
      Trans.close();
    }
    else{
      Trans.open();
    }
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Clamp.open();
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);
  thread ooshiny(oooShiny);
  ooshiny.detach();
  while(1){
    Brain.Screen.printAt(20, 20, "speedmod = %f", speedmod);
    Brain.Screen.printAt(20, 40, "Optical Brightness: %f", Optical.brightness());
  }
}
