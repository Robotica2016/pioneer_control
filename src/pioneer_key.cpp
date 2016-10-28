/*
 pioneer_key does some basic keyboard pioneer control.
 Copyright (C) 2013  Rafael Berkvens rafael.berkvens@ua.ac.be

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* Code from http://irawiki.disco.unimib.it/irawiki/index.php/Teleoperate_p3dx_with_Ros_and_Aria */
/* Code from http://ros.org/doc/groovy/api/turtlesim/html/teleop__turtle__key_8cpp_source.html */

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_0 0x30

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

class RobotDriver
{
private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "cmd_vel" topic to issue commands
  ros::Publisher cmd_vel_pub_;

  double linear_, angular_, l_scale_, a_scale_;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh) :
      linear_(0), angular_(0), l_scale_(0.1), a_scale_(0.1)
  {
    nh_ = nh;

    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/p3dx/cmd_vel", 1);
  }

  ~RobotDriver()
  {
    quit(0);
  }

  //! Loop forever while sending drive commands based on keyboard input
  void driveKeyboard()
  {
    char c;
    bool dirty = false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");

    while (nh_.ok())
    {
      // get the next event from the keyboard
      if (read(kfd, &c, 1) < 0)
      {
        perror("read():");
        exit(-1);
      }

      //linear_ = angular_ = 0;
      ROS_DEBUG("value: 0x%02X\n", c);

      switch (c)
      {
        case KEYCODE_L:
          ROS_DEBUG("LEFT");
          angular_ += 1.0;
          dirty = true;
          break;
        case KEYCODE_R:
          ROS_DEBUG("RIGHT");
          angular_ -= 1.0;
          dirty = true;
          break;
        case KEYCODE_U:
          ROS_DEBUG("UP");
          linear_ += 1.0;
          dirty = true;
          break;
        case KEYCODE_D:
          ROS_DEBUG("DOWN");
          linear_ -= 1.0;
          dirty = true;
          break;
        case KEYCODE_Q:
          ROS_DEBUG("QUIT");
          nh_.shutdown();
          break;
        case KEYCODE_0:
          ROS_DEBUG("BREAK");
          linear_ = 0.0;
          angular_ = 0.0;
          dirty = true;
          break;
      }

      geometry_msgs::Twist base_cmd;
      base_cmd.angular.z = a_scale_ * angular_;
      ROS_DEBUG_STREAM("angular: " << angular_);
      base_cmd.linear.x = l_scale_ * linear_;
      ROS_DEBUG_STREAM("linear: " << linear_);
      if (dirty == true)
      {
        cmd_vel_pub_.publish(base_cmd);
        dirty = false;
      }
    }
  }

};

int main(int argc, char **argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  driver.driveKeyboard();
}

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
