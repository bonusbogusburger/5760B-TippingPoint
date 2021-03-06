#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;
limit HookLimit(Brain.ThreeWirePort.A); //A global instance of our limit switch because why
pneumatics LeftClamp(Brain.ThreeWirePort.H);
pneumatics RightClamp(Brain.ThreeWirePort.G);
pneumatics RRelease(Brain.ThreeWirePort.F);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}