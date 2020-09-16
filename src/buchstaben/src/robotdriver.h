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

#ifndef ROBOTDRIVER_H
#define ROBOTDRIVER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <turtlesim/SetPen.h>
#include <math.h>
#include <string>

#define PI 3.1415

class RobotDriver
{
private:
    ros::NodeHandle _nh;
    ros::Publisher _velocityCommandPublisher; // Publisher for sending the speed parameters
    ros::Subscriber _currentPoseSubscriber;   // Subscriber to update the position data
    ros::ServiceClient _serviceClient;        // Service Client to control the robots pen
    turtlesim::Pose _currentPosition;         // current coordinate parameters and speed of the robot
    bool _initPositionAvailable;              // flag: position data available

public:
    RobotDriver(ros::NodeHandle &nh, std::string turtleName, double loopRate);

    void CurrentPoseCallback(const turtlesim::Pose::ConstPtr &msg);
    void DeactivatePen();
    void ActivatePen();
    bool Forward(double distance, double velocity);
    void TurnClockwise(double degree, double velocity);
    void TurnCounterClockwise(double degree, double velocity);
    bool TurnToTheta(double theta, double velocity);
    bool GoToPosition(double x, double y, double theta, double velocity);
    bool ForwardAndTurn(double radius, double degree, double velocity);
};

#endif