/*
    pioneer_joy does some basic joystick pioneer control. 
    Copyright (C) 2013  Rafael Berkvens rafael.berkvens@uantwerpen.be

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

#include <cmath>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Twist.h>

class PioneerControl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &laser);
  void sonarCallback(const sensor_msgs::PointCloud::ConstPtr &sonar);

  ros::NodeHandle node_handle_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber laser_sub_;
  ros::Subscriber sonar_sub_;
  geometry_msgs::Twist cmd;

  int linear_, angular_;
  double l_scale_, a_scale_;
  // PlayStation controller buttons (one l_two_ is currently in use):
  int l_one_, l_two_, r_one_, r_two_;

public:
  PioneerControl();
  ~PioneerControl();
};

void PioneerControl::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  double adjustable_a_scale = a_scale_, adjustable_l_scale = l_scale_;
  if(joy->axes[l_two_] < 0.0)
  {
    adjustable_l_scale *= std::abs(joy->axes[l_two_]) * 10.0;
    adjustable_a_scale *= std::abs(joy->axes[l_two_]) * 5.0;
  }
  ROS_DEBUG_STREAM("adjustable_l_scale: " << adjustable_l_scale);
  cmd.angular.z = adjustable_a_scale * joy->axes[angular_];
  cmd.linear.x = adjustable_l_scale * joy->axes[linear_];
  vel_pub_.publish(cmd);
}

PioneerControl::PioneerControl()
{
  // Say what and where the publisher is publishing messages.
  vel_pub_ = node_handle_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel",
             1);

  // Say where to get messages.
  joy_sub_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 10,
             &PioneerControl::joyCallback, this);

  // Set the default message values to 0.0
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  // Get parameters from ROS
  ros::NodeHandle n_private( "~" );
  n_private.param("axis_linear", linear_, 1);
  n_private.param("axis_angular", angular_, 0);
  n_private.param("left_front", l_two_, 12);
  n_private.param("left_front", r_two_, 13);
  n_private.param("left_front", l_one_, 14);
  n_private.param("left_front", r_one_, 15);
  n_private.param("scale_angular", a_scale_, 0.5);
  n_private.param("scale_linear", l_scale_, 0.25);
}

PioneerControl::~PioneerControl()
{
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pioneer_control");

  PioneerControl pioneer_control;

  ros::spin();
}

// kate: indent-mode cstyle; indent-width 2; replace-tabs on; 
