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

#include "robotdriver.h"

/**
 * 
 * @brief Rotate robot clockwise
 * 
 * @param degree angle in degrees
 * @param velocity angular velocity in rad/s
 * 
*/
void RobotDriver::TurnClockwise(double degree, double velocity)
{
    ROS_INFO("Turn clockwise phi=%f Deg | v=%f rad/s", degree, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    double radAngle = fabs(degree) * PI / 180.0; // convert from deg to rad
    double movedAngle = 0.0;                     // swept angle
    double lastAngle = _currentPosition.theta;   // angle of last position

    geometry_msgs::Twist velocityCommand; // velocity of the robot

    // because of rotation around z-axis -> all other set to 0
    velocityCommand.linear.x = 0;
    velocityCommand.linear.y = 0;
    velocityCommand.linear.z = 0;
    velocityCommand.angular.x = 0;
    velocityCommand.angular.y = 0;

    // clockwise angular velocity (absolute value to not change direction)
    velocityCommand.angular.z = -1 * fabs(velocity);

    // rotation up to angle swept over
    while (_nh.ok() && ros::ok() && (movedAngle < radAngle))
    {
        ros::spinOnce();
        _velocityCommandPublisher.publish(velocityCommand);

        // angle of rotation sweeps the x-axis from the 1. quardrant to the 4. quadrant
        if (_currentPosition.theta > lastAngle)
        {
            // swept angle = angle of 1. Quard. + difference between full circle and current angle
            movedAngle += lastAngle + ((2 * PI) - _currentPosition.theta);
        }
        else
        {
            movedAngle += lastAngle - _currentPosition.theta;
        }

        lastAngle = _currentPosition.theta;
    }

    // stop z-axis rotation
    velocityCommand.angular.z = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    return;
}

/**
*
* @brief Rotate robot counterclockwise
*
* @param degree angle in degrees
* @param velocity angular velocity in rad/s
*
*/
void RobotDriver::TurnCounterClockwise(double degree, double velocity)
{
    ROS_INFO("Turn counterclockwise phi=%f Deg | v=%f rad/s", degree, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    double radAngle = fabs(degree) * PI / 180.0; // convert from deg to rad
    double lastAngle = _currentPosition.theta;   // angle of last position
    double movedAngle = 0.0;                     // swept angle

    geometry_msgs::Twist velocityCommand; // Geschwindigkeit des Roboters

    // because of rotation around z-axis -> all other set to 0
    velocityCommand.linear.x = 0;
    velocityCommand.linear.y = 0;
    velocityCommand.linear.z = 0;
    velocityCommand.angular.x = 0;
    velocityCommand.angular.y = 0;
    velocityCommand.angular.z = fabs(velocity);

    // rotation up to angle swept over
    while (_nh.ok() && ros::ok() && (movedAngle < radAngle))
    {
        ros::spinOnce();
        _velocityCommandPublisher.publish(velocityCommand);

        // angle of rotation sweeps the x-axis from the 4. quardrant to the 1. quadrant
        double rest = 0.0;
        if (lastAngle > _currentPosition.theta)
        {
            rest = (2 * PI) - lastAngle; // Restwinkel vor dem Übertrit in den 1. Quadranten
            movedAngle += rest + _currentPosition.theta;
        }
        else
        {
            movedAngle += (_currentPosition.theta - lastAngle);
        }

        lastAngle = _currentPosition.theta;
    }

    // stop z-axis rotation
    velocityCommand.angular.z = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    return;
}

/**
*
* @brief Callback function - received current robot position
*
* This function is called by the ROS Subscriber and updates the current
* position, angle of rotation and speed parameters of the robot.
* The transmitted angle theta gets converted into an angle between 0 and 2 pi.
*
* @param msg ROS message format Pose
*
*/
void RobotDriver::CurrentPoseCallback(const turtlesim::Pose::ConstPtr &msg)
{
    _currentPosition = *msg;
    _initPositionAvailable = true;

    // calculate angle between 0 and 2 pi
    if (_currentPosition.theta < 0.0)
    {
        // convert angle between 0 and 2 pi
        _currentPosition.theta = (2 * PI) + _currentPosition.theta;
    }

    return;
}

/**
 * 
 * @brief Constructor
 * Initialization of the service client to control the pen. Initialization of the
 * subscriber in order to receive position data from the robot. As well as
 * initializing the command publisher to send speed parameters to the robot.
 * 
 * @param nh reference to NodeHandle
 * @param turtleName name of the robot ROS topic
 * @param loopRate ROS sampling rate
 * 
*/
RobotDriver::RobotDriver(ros::NodeHandle &nh, std::string turtleName, double loopRate)
{
    _nh = nh;
    _velocityCommandPublisher = _nh.advertise<geometry_msgs::Twist>("/" + turtleName + "/cmd_vel", 1);
    _currentPoseSubscriber = _nh.subscribe("/" + turtleName + "/pose", 5, &RobotDriver::CurrentPoseCallback, this);
    _serviceClient = _nh.serviceClient<turtlesim::SetPen>("/" + turtleName + "/set_pen");

    ros::Rate loop_rate(loopRate);
    _initPositionAvailable = false;
}

/**
 * 
 * @brief Deactivate robot pen
 * 
 * 
*/
void RobotDriver::DeactivatePen()
{
    ROS_INFO("Deactivate pen");

    turtlesim::SetPen srv;
    srv.request.off = 1;
    _serviceClient.call(srv);
}

/**
 * 
 * @brief Activate robot pen
 * 
 * Activates the robot's pen and sets
 * the line width to 5
 * 
*/
void RobotDriver::ActivatePen()
{
    ROS_INFO("Activate pen");

    turtlesim::SetPen srv;
    srv.request.off = 0;
    srv.request.width = 5;
    _serviceClient.call(srv);
}

/**
 * 
 * @brief Linear motion forward (relative)
 * 
 * Bewegt den Roboter um eine relative Einheit nach vorn.
 * (Mathematische Zusammenhänge sind in der Dokumentation im Kapitel 4.5 zu finden)
 * 
 * @param distance distance in m
 * @param velocity velocity in m/s
 * 
*/
bool RobotDriver::Forward(double distance, double velocity)
{
    //std::cout << "> Forward l=" << distance << "m | v=" << velocity << "m/s" << std::endl;
    ROS_INFO("Forward l=%f m | v=%f m/s", distance, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    geometry_msgs::Twist velocityCommand; // velocity parameters of the robot

    // set all motion directions to 0
    velocityCommand.linear.x = velocityCommand.linear.y = velocityCommand.angular.z = 0;

    //
    // calculate target coordinate
    //

    double target_x = _currentPosition.x; // initialization with current x-position (world coordinate)
    double target_y = _currentPosition.y; // initialization with current y-position (world coordinate)

    // current angle located in 1. Quad.
    if (_currentPosition.theta >= 0.0 && _currentPosition.theta <= PI)
    {
        target_x += distance * cos(_currentPosition.theta);
        target_y += distance * sin(_currentPosition.theta);
    }
    // current angle located in 2. Quad.
    else if (_currentPosition.theta <= (PI - 0.01))
    {
        target_x += -1 * distance * cos(PI - _currentPosition.theta);
        target_y += -1 * distance * sin(PI - _currentPosition.theta);
    }
    // current angle located in 3. Quad.
    else if (_currentPosition.theta <= (3 / 2.0 * PI))
    {
        target_x += -1 * distance * sin((3 / 2.0 * PI) - _currentPosition.theta);
        target_y += -1 * distance * cos((3 / 2.0 * PI) - _currentPosition.theta);
    }
    // current angle located in 4. Quad.
    else
    {
        target_x += distance * cos(2 * PI - _currentPosition.theta);
        target_y += -1 * distance * sin(2 * PI - _currentPosition.theta);
    }

    // linear movement until target position is reached with a tolerance of 0.01 m
    while (_nh.ok() && ros::ok() && (fabs(_currentPosition.x - target_x) > 0.01 || fabs(_currentPosition.y - target_y) > 0.01))
    {
        ros::spinOnce();

        velocityCommand.linear.x = fabs(velocity);
        _velocityCommandPublisher.publish(velocityCommand);
    }

    // stop robot
    velocityCommand.linear.x = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    return true;
}

/**
 * 
 * @brief Rotation to absolute angle
 * 
 * Moves the robot to a specified angle.
 * The robot rotates in the shortest direction of
 * rotation until the angle is reached.
 * 
 * @param theta Absolute angle at which the robot should turn in degrees
 * @param velocity angular velocity in rad/s
*/
bool RobotDriver::TurnToTheta(double theta, double velocity)
{
    //std::cout << "> Turn to theta theta=" << theta << "Deg | v=" << velocity << "rad/s" << std::endl;
    ROS_INFO("Turn to theta=%f Deg | v=%f rad/s", theta, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    double radAngle = theta * PI / 180.0;        // convert from deg to rad
    double actualTheta = _currentPosition.theta; // current rotation angle
    geometry_msgs::Twist velocityCommand;        // robot velocity parameters

    // calculate shortest rotation direction
    bool clockwise = true;
    if (fabs(actualTheta - radAngle) < fabs(actualTheta + (2 * PI - radAngle)))
    {
        clockwise = false;
    }

    // rotate robot to specific angle until reaching with a tolerance of 0.01 rad
    while (_nh.ok() && ros::ok() && fabs(radAngle - actualTheta) > 0.01)
    {
        ros::spinOnce();
        actualTheta = _currentPosition.theta;

        if (clockwise)
            velocityCommand.angular.z = -1 * velocity;
        else
            velocityCommand.angular.z = velocity;

        _velocityCommandPublisher.publish(velocityCommand);
    }

    // stop robot
    velocityCommand.angular.z = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    return true;
}

/**
 * 
 * @brief Linear and angular movement to absolute position
 * 
 * Calculation and rotation to target position angle. Then linear
 * movement to target coordinates.
 * 
 * @param x absolute x-coordinate
 * @param y absolute y-coordinate
 * @param theta aboslute angle in degrees
 * @param velocity linear and angular velocity in m/s and rad/s
 *  
*/
bool RobotDriver::GoToPosition(double x, double y, double theta, double velocity)
{
    ROS_INFO("Go to position P(%f|%f) v=%f m/s", x, y, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    // calculation of the relative difference between target and current coordinates
    double x_coord = x - _currentPosition.x;
    double y_coord = y - _currentPosition.y;

    // needed absolute angle of the robot to get to target position
    double phi = atan2(y_coord, x_coord);

    geometry_msgs::Twist velocityCommand; // velocity parameters of the robot

    // rotate robot to specific angle until reaching with a tolerance of 0.01 rad
    while (_nh.ok() && ros::ok() && fabs(phi - _currentPosition.theta) > 0.01)
    {
        ros::spinOnce();
        velocityCommand.angular.z = velocity;
        _velocityCommandPublisher.publish(velocityCommand);
    }

    // stop robot
    velocityCommand.angular.z = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    // linear movement forward to reach target coordinate with a tolerance of 0.01 m
    // it is assumed that, depending on the angle and the x-coordinate, the
    // y-coordinate does not have to be checked anymore.
    while (_nh.ok() && ros::ok() && fabs(x - _currentPosition.x) > 0.01)
    {
        ros::spinOnce();
        velocityCommand.linear.x = velocity;
        _velocityCommandPublisher.publish(velocityCommand);
    }

    // stop robot
    velocityCommand.linear.x = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    // rotate to absolute angle
    this->TurnToTheta(theta, velocity);

    return true;
}

/**
 * 
 * @brief Circular path movement
 * 
 * Robot movement on a circular path with given radius and the
 * angle of the circular  arc. (moving counterclockwise)
 * 
 * @param radius radius of the circular path
 * @param degree angle of the circular arc
 * @param velocity angle and linear velocity in rad/s and m/s
 * 
*/
bool RobotDriver::ForwardAndTurn(double radius, double degree, double velocity)
{
    //std::cout << "> Forward and turn r=" << radius << " m phi=" << degree << " Grad v=" << velocity << "m/s" << std::endl;
    ROS_INFO("Forward and turn r=%f m phi=%f Deg. v=%f m/s", radius, degree, velocity);

    // wait until first callback with position parameters of the robot is available
    while (!_initPositionAvailable)
        ros::spinOnce();

    double radAngle = degree * PI / 180.0;     // convert from deg to rad
    double lastAngle = _currentPosition.theta; // angle of last position
    double movedAngle = 0.0;                   // swept angle of the robot
    geometry_msgs::Twist velocityCommand;      // velocity parameters of the robot

    //circular path movement up to the angle of the circular path
    while (_nh.ok() && ros::ok() && movedAngle < radAngle)
    {
        ros::spinOnce();
        velocityCommand.angular.z = velocity / radius;
        velocityCommand.linear.x = velocity;
        _velocityCommandPublisher.publish(velocityCommand);

        // angle of rotation sweeps the x-axis from 4. quad. to 1. quad.
        if (lastAngle > _currentPosition.theta)
        {
            double rest = 2 * PI - lastAngle; // remaining angle before exceeding in the 1st quadrant
            movedAngle += rest + _currentPosition.theta;
        }
        else
        {
            movedAngle += _currentPosition.theta - lastAngle;
        }

        lastAngle = _currentPosition.theta;
    }

    // stop robot
    velocityCommand.angular.z = 0;
    velocityCommand.linear.x = 0;
    _velocityCommandPublisher.publish(velocityCommand);

    return true;
}