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

double speeds [10] = {1.00, .90, .80, .70, .60, .50, .40, .30, .20, .10};
double speed = 0.3;
int speedSwitcher(){ //task that switches speed hallelujah
  return 1;
}

double stickTest3;
double stickTest1;
int time3 = 0;
bool stopit = false;
int timer3(){
  time3 = 0;
  while(1){
    wait(1, msec);
    time3 += 1;
    if(stopit == true){
      break;
    }
  }
  return time3;
}
void driver(){
  double lastStick3 = 0;
  double lastStick1 = 0;
  while(1){
    Brain.Screen.clearLine(20, 60);
    stickTest3 = floor(Controller.Axis3.position()/10);
    stickTest1 = floor(Controller.Axis1.position()/10);
    if(stickTest3 != 0){
      double mod = Controller.Axis1.position()*0.65;
      Left.spin(fwd, stickTest3*10+mod, pct);
      Right.spin(fwd, stickTest3*10-mod, pct);
      if(lastStick3 != stickTest3){
        std::cout.precision(3); //the next line has to be very long i am so sorry
        std::cout << "Accelerated from " << lastStick3*10 << " to " << stickTest3*10 << " (GPS X: " << GPS.xPosition(inches) << " Y: " << GPS.yPosition(inches) << ") " << time3 << "ms" << std::endl;
        time3 = 0;
      }
      lastStick3 = stickTest3;
    }
    else if(stickTest1 != 0){
      Left.spin(fwd, stickTest1*10, pct);
      Right.spin(reverse, stickTest1*10, pct);
    }
    else{
      stopit = true;
      DTrain.stop(brake);
    }
  }
}

void auton(){
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
