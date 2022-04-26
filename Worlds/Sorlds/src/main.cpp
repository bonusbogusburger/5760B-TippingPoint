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
motor CL(PORT8, true);
motor IL(PORT1);

//Expanders
triport Expander1(PORT6);

//Sensors
gps GPS(PORT5);
vex::distance Distance(PORT3);
potV2 Pot1(Expander1.G);

//Pneumatics
pneumatics Clamp(Brain.ThreeWirePort.B);
pneumatics ILift(Brain.ThreeWirePort.E);
pneumatics TransL(Brain.ThreeWirePort.D);
pneumatics TransR(Brain.ThreeWirePort.A);

//ooo shiny
led Green1(Expander1.A);
//led Green2(Expander1.B);
led Yellow1(Expander1.B);
//led Yellow2(Expander1.D);
led Red1(Expander1.C);
//led Red2(Expander1.F);

/*void oooShiny(){ //haha funny led
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
}*/

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

bool automatic = true;
bool actuate = false;
float speedmod = 1;
float watts = 0;
void autoshifter(){
  while(1){
    watts = (DR.power()+DL.power())/2;
    if(watts < 1.5*3){
      speedmod = 1;
      actuate = false;
      Green1.on();
      Yellow1.off();
      Red1.off();
    }
    else if(watts<3.5*3){
      speedmod = 0.5;
      actuate = false;
      Green1.off();
      Yellow1.on();
      Red1.off();
    }
    else if(watts<3.5*3){
      speedmod = 1;
      actuate = true;
      Green1.off();
      Yellow1.off();
      Red1.on();
    }
    else if(watts<4*3){
      speedmod = 0.5;
      actuate = true;
      Green1.on();
      Yellow1.on();
      Red1.on();
    }
    wait(0.0000000000000000001,msec);
  }
}

double lastStick3 = 0;
double lastStick2 = 0;
int counter = 3;
void autoshift2(){ //hoo boy
  counter = 3;
  while(1){
    double lspct = Cont1.Axis3.position();
    double rspct = Cont1.Axis2.position();
    double stickChange3 = abs(Cont1.Axis3.position()) - lastStick3;
    double stickChange2 = abs(Cont1.Axis2.position()) - lastStick2;
    if(stickChange3 > 0 or stickChange2 > 0){
      wait(100, msec);
    }
    double lpct = DL.velocity(pct)/(lspct*speedmod)*100;
    double rpct = DR.velocity(pct)/(rspct*speedmod)*100;
    if(lpct < 10 or rpct < 10){
      //TransL.open();
      //TransR.open();
    }
    else if(lpct > 50 or lpct < 1 or stickChange3 < 0 or rpct > 50 or rpct < 1 or stickChange2 < 0){
      //TransL.close();
      //TransR.close();
    }

    if(lpct > 80 or rpct > 80){
      TransL.close();
      TransR.close();
      speedmod = 1;
    }
    else if(lpct > 50 or rpct > 50){
      TransL.close();
      TransR.close();
      speedmod = 0.5;
    }
    else if(lpct > 30 or rpct > 30){
      TransL.open();
      TransR.open();
      speedmod = 1;
    }
    else if(lpct > 10 or rpct > 10){
      TransL.open();
      TransR.open();
      speedmod = 0.5;
    }

    /*if(rpct < 10){
      TransR.open();
    }
    else if(rpct > 50 or rpct < 1 or stickChange2 < 0){
      TransR.close();
    }
    }*/
    lastStick3 = abs(Cont1.Axis3.position());
    lastStick2 = abs(Cont1.Axis2.position()); 
  }
  wait(1, sec);
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
    else if(Cont1.ButtonX.pressing() & (cycle == 3)){
      speedmod = 0.5;
      actuate = true;
      cycle = 0;
    }
    wait(0.25, sec);
  }
}

void transToggle(){
  actuate = false;
  while(1){
    if(Cont1.ButtonRight.pressing()){
      actuate = !actuate;
      wait(0.5, sec);
    }
    wait(5, msec);
  }
}

//thread parent, controls automatic and manual
void weaver(){
  //sense forchanges and record them
  int mode = 1;
  int cntmode = 1;
  vex::thread autoshift(autoshift2);
  autoshift.detach();
  vex::thread manual(transToggle);
    manual.detach();
    autoshift.interrupt();
  while(1){
  if(Cont1.ButtonLeft.pressing() & (mode == 0)){
    mode = 1;
    automatic = false;
  }
  else if(Cont1.ButtonUp.pressing() & (mode == 1)){
    mode = 0;
    automatic = true;
  }
  wait(0.0000000001, sec);
  if((!automatic) & (cntmode == 1)){
    vex::thread manual(manualshifter);
    manual.detach();
    autoshift.interrupt();
    cntmode = 0;
  }
  else if (automatic & (cntmode == 0)){
  vex::thread autoshift(autoshift2);
  autoshift.detach();
  manual.interrupt();
  cntmode = 1;
  }
  }
}

bool bactuate;
void backToggle(){
  bactuate = false;
  while(1){
    if(Cont1.ButtonR2.pressing()){
      bactuate = !bactuate;
      wait(0.5, sec);
    }
    wait(5, msec);
  }
}

bool cactuate;
void clampToggle(){
  cactuate = true;
  while(1){
    if(Cont1.ButtonY.pressing()){
        cactuate = !cactuate;
      wait(0.5, sec);
    }
    wait(5, msec);
  }
}

void driver(){
  //thread weave(weaver);
  thread manual(transToggle);
  thread btoggle(backToggle);
  thread ctoggle(clampToggle);
  //weave.detach();
  manual.detach();
  btoggle.detach();
  ctoggle.detach();
  while(1){
    if(fabs((floor(Cont1.Axis3.position()/10)*10)) > 0 or fabs((floor(Cont1.Axis2.position()/10)*10)) > 0){
      gvspin(DL, Cont1.Axis3.position()*speedmod);
      gvspin(DR, Cont1.Axis2.position()*speedmod);
    }
    else{
      DT.stop(brake);
    }

    if(Cont1.ButtonB.pressing()){
      vspin(CL, 100);
    }
    else if(Cont1.ButtonDown.pressing()){
      vspin(CL, -100);
    }
    else{
      CL.stop(hold);
    }

    if(Cont1.ButtonR1.pressing()){
      vspin(IL, 75);
    }
    else if(Cont1.ButtonL1.pressing()){
      vspin(IL, -75);
    }
    else{
      IL.stop(hold);
    }

    if(cactuate == true){
      Clamp.open();
    }
    else if(cactuate == false){
      Clamp.close();
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

void auton(){
  wait(1, sec);
  CL.spinFor(reverse, 1, rev, false);
  Clamp.open();
  wait(50, msec);
  gvspin(DL, 100);
  gvspin(DR, 100);
  waitUntil(Distance.objectDistance(mm) < 25);
  Clamp.close();
  wait(50, msec);
  CL.stop(hold);
  gvspin(DL, 50);
  gvspin(DR, 50);
  waitUntil(GPS.xPosition(inches) > 20);
  DT.stop(brake);
  wait(0.5, sec);
  gvspin(DL, -50);
  gvspin(DR, 50);
  waitUntil(GPS.heading() < 91);
  DT.stop(brake);
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Clamp.open();
  GPS.calibrate();
  TransL.close();
  TransR.close();
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);
  while(1){
    Brain.Screen.setFillColor(black);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(monoM);
    Brain.Screen.printAt(20, 20, "speedmod = %f", speedmod);
    Brain.Screen.printAt(20, 40, "DL1 Temp: %f", DL1.temperature(celsius));
    Brain.Screen.printAt(20, 60, "DL2 Temp: %f", DL2.temperature(celsius));
    Brain.Screen.printAt(20, 80, "DL3 Temp: %f", DL3.temperature(celsius));
    Brain.Screen.printAt(20, 100, "DR1 Temp: %f", DR1.temperature(celsius));
    Brain.Screen.printAt(20, 120, "DR2 Temp: %f", DR2.temperature(celsius));
    Brain.Screen.printAt(20, 140, "DR3 Temp: %f", DR3.temperature(celsius));
    Brain.Screen.printAt(20, 160, "watts: %f", watts);
    Brain.Screen.printAt(20, 180, "Distance: %f mm", Distance.objectDistance(mm));
    Brain.Screen.setFont(propXXL);
    Brain.Screen.setFillColor(red);
    Brain.Screen.setPenColor(black);
    Brain.Screen.printAt(280, 105, "5760B");
    cout << "counter = " << counter << endl;
    //cout << "L1 Velocity = " << DL1.velocity(pct) << endl;
  }
}
