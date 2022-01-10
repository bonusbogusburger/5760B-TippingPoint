#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
 //A global instance of our limit switch because why
pneumatics LeftClamp(Brain.ThreeWirePort.H);
pneumatics RightClamp(Brain.ThreeWirePort.G);
pneumatics RRelease(Brain.ThreeWirePort.F);
// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}