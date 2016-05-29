/*
This file is part of Yamabros.

Yamabros is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Yamabros is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Yamabros. If not, see <http://www.gnu.org/licenses/>.
*/

#include <yamabros/spur/actions_ros.h>

#include <yamabros/settings.h>
#include <yamabros/yamabros.h>

#include <tf/transform_broadcaster.h>

#include <boost/bind.hpp>

#include <cmath>

namespace yamabros
{

namespace spur
{

ActionsROS::ActionsROS():
  Actions(),
  node_(),
  tt1_(0.0)
{
  cmd_vel_ = node_.advertise<geometry_msgs::Twist>(settings::spur::cmd_vel(), 1);
  odom_ = node_.subscribe<nav_msgs::Odometry>(settings::spur::odom(), 1, &ActionsROS::odom_callback, this);
}

void ActionsROS::odom_callback(const nav_msgs::OdometryConstPtr& odometry)
{
  Pose &pose = this->pose();
  pose.x = odometry->pose.pose.position.x;
  pose.y = odometry->pose.pose.position.y;

  /*
  The management of pose orientation is more complicated for the ROS implementation
  because yaw is reported as an angle between -pi and pi, but what we really want
  is an absolute value that increases / decreases indefinitely with each additional
  turn. In order to convert between the two formats it's necessary to keep track of
  the instantaneous angle and accumulate changes over time.
  */

  double t = tf::getYaw(odometry->pose.pose.orientation);

  // If the difference between previous and current orientations is greater than pi,
  // it means there was an overflow from pi to -pi or vice-versa.
  double dt = t - tt1_;
  double dm = fabs(dt);
  if (dm >= M_PI)
    dt = copysign(1.0, dt) * (dm - 2 * M_PI);

  pose.t += dt;
  tt1_ = t;

  ROS_INFO_STREAM("Pose : (" << pose.x << ", " << pose.y << ", " << pose.t << ")");
}

void ActionsROS::approach(double x, double y, double t)
{
  // TODO: substitute proper implementation
  straight();
}

void ActionsROS::brake()
{
  set(0, 0);
}

void ActionsROS::circle(double x, double y, double r)
{

  // TODO: substitute proper implementation

  // The angular speed over a circle of radius r is the ratio
  // between the radius and the linear speed.
  double v = settings::spur::lin_vel();
  double w = v / r;

  set(v, w);
}

void ActionsROS::coast()
{
  set(0, 0);
}

void ActionsROS::set(double v, double w)
{
  geometry_msgs::Twist twist;
  twist.linear.x = v;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = w;

  cmd_vel_.publish(twist);
}

void ActionsROS::spin(double t)
{
  double w = copysign(1.0, t) * settings::spur::ang_vel();

  set(0, w);
}

void ActionsROS::straight()
{
  double v = settings::spur::lin_vel();

  set(v, 0);
}

} // namespace spur

} // namespace yamabros
