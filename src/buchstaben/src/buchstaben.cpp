/**
* Copyright 2019 Jean-Marcel Herzog
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
* associated documentation files (the "Software"), to deal in the Software without restriction,
* including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense
* and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do
* so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
* INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
* PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR 
* COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
* AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
* WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <iostream>
#include <ros/ros.h>

#include "robotdriver.h"
#include "draw.h"

#define VELOCITY_DRAW 0.75       // velocity in m/s while drawing
#define VELOCITY_QUICKDRIVE 0.75 // velocity in m/s while moving between the coordinates or between letters to draw.
#define TURTLENAME "turtle1"     // name of the robot topic
#define LOOPRATE 10              // ROS sampling rate

/**
 * 
 * @brief Entry point
 * 
 * Initialization of a ROS node and draw letters.
 * 
 * @param argc argument counter
 * @param argv argument vector
*/
int main(int argc, char **argv)
{
    // ROS node initialization
    ros::init(argc, argv, "buchstaben");
    ros::NodeHandle nodeHandle;

    RobotDriver driver(nodeHandle, TURTLENAME, LOOPRATE);
    Draw paint(&driver);

    // move to start position P(3|10)
    driver.DeactivatePen();
    driver.GoToPosition(3.0, 10.0, 180.0, VELOCITY_QUICKDRIVE);

    // draw letter F
    paint.paint_F(VELOCITY_DRAW);

    // move to next start position of next letter
    driver.TurnCounterClockwise(90.0, VELOCITY_QUICKDRIVE);
    driver.Forward(2.0, VELOCITY_QUICKDRIVE);
    driver.TurnCounterClockwise(90.0, VELOCITY_QUICKDRIVE);
    driver.Forward(0.5, VELOCITY_QUICKDRIVE);
    driver.TurnClockwise(180.0, VELOCITY_QUICKDRIVE);

    // draw letter J
    paint.paint_J(VELOCITY_DRAW);

    // move to next start position of next letter
    driver.TurnClockwise(180.0, VELOCITY_QUICKDRIVE);
    driver.Forward(2.0, VELOCITY_QUICKDRIVE);

    // draw letter D
    paint.paint_D(VELOCITY_DRAW);

    // move away from drawing
    driver.TurnCounterClockwise(90.0, VELOCITY_QUICKDRIVE);
    driver.Forward(3.0, VELOCITY_QUICKDRIVE);

    return 0;
}