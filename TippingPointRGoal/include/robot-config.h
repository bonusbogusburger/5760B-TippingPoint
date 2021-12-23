using namespace vex;

extern brain Brain;
extern limit HookLimit;
extern pneumatics RightClamp;
extern pneumatics LeftClamp;
extern pneumatics RRelease;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
