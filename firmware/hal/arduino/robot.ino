// Generated sketch layout: `make arduino` copies the C sources next to this
// file into src/. Open build/arduino/robot/robot.ino in Arduino IDE.
#include "src/hal.h"
#include "src/robot.h"

void setup()
{
    hal_init();
    robot_setup();
}

void loop()
{
    robot_loop();
}
