/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature SIG_1 = vex::vision::signature (1, -2635, -1831, -2233, 7115, 9883, 8499, 4.2, 0);
vex::vision::signature SIG_2 = vex::vision::signature (2, 6449, 8513, 7481, -613, 1, -306, 4.2, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 601, 1649, 1126, -3077, -2585, -2832, 3.2, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision FVis = vex::vision (vex::PORT2, 50, SIG_1, SIG_2, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/