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

#include "draw.h"

/**
 * 
 * @brief Constructor
 * 
 * @param driver pointer to RobotDriver object
*/
Draw::Draw(RobotDriver *driver)
{
    _driver = driver;
}

/**
 * 
 * @brief Draw letter F
 * 
 * @param velocity velocity of the robot
*/
void Draw::paint_F(double velocity)
{
    _driver->ActivatePen();
    _driver->Forward(1.0, velocity);
    _driver->TurnCounterClockwise(90.0, velocity);
    _driver->Forward(1.0, velocity);
    _driver->TurnCounterClockwise(90.0, velocity);
    _driver->Forward(1.0, velocity);
    _driver->TurnClockwise(180.0, velocity);
    _driver->Forward(1.0, velocity);
    _driver->TurnCounterClockwise(90.0, velocity);
    _driver->Forward(1.0, velocity);
    _driver->DeactivatePen();
}

/**
 * 
 * @brief Draw letter J
 * 
 * @param velocity velocity of the robot
 * 
*/
void Draw::paint_J(double velocity)
{
    _driver->ActivatePen();
    _driver->ForwardAndTurn(0.5, 180.0, velocity);
    _driver->Forward(1.5, velocity);
    _driver->TurnCounterClockwise(90.0, velocity);
    _driver->Forward(1.0, velocity);
    _driver->DeactivatePen();
}

/**
 * 
 * @brief Draw letter D
 * 
 * @param velocity velocity of the robot
 * 
*/
void Draw::paint_D(double velocity)
{
    _driver->ActivatePen();
    _driver->TurnClockwise(90.0, velocity);
    _driver->Forward(2.0, velocity);
    _driver->TurnCounterClockwise(90.0, velocity);
    _driver->Forward(0.25, velocity);
    _driver->ForwardAndTurn(1.0, 180.0, velocity);
    _driver->Forward(0.25, velocity);
    _driver->DeactivatePen();
}