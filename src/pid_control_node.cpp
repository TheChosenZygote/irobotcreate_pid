// A barebones angular and linear PID controller for the Roomba
// Unoptimized, controllers work separately, use only for debugging

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <tf/tf.h>
#include <math.h>

int rate = 10;

float waypoint_x[5];
float waypoint_y[5];
int waypoints = 5;
int current = 0;
bool waypoint_reached = false;

float x_pos = 0;
float y_pos = 0;
float yaw = 0;

float kp_lin = 2.5;
float ki_lin = 0.3;
float kd_lin = 0.1;

float kp_ang = 2.2;
float ki_ang = 0.25;
float kd_ang = 0.005;

float integral_lin = 0;
float integral_ang_pos = 0;
float integral_ang_neg = 0;

float prior_lin = 0;
float prior_ang_pos = 0;
float prior_ang_neg = 0;

float min_output_lin = -0.5;
float max_output_lin = 0.5;
float min_output_ang = -4.25;
float max_output_ang = 4.25;

void callBack(const nav_msgs::Odometry msg)
{
  x_pos = msg.pose.pose.position.x;
  y_pos = msg.pose.pose.position.y;

  tf::Pose pose;
  tf::poseMsgToTF(msg.pose.pose, pose);
  yaw = tf::getYaw(pose.getRotation());
}

int main(int argc, char **argv)
{
  waypoint_x[0] = 5.0;
  waypoint_x[1] = -4.0;
  waypoint_x[2] = 1.0;
  waypoint_x[3] = -7.0;
  waypoint_x[4] = 8.0;

  waypoint_y[0] = 6.0;
  waypoint_y[1] = 3.0;
  waypoint_y[2] = -4.0;
  waypoint_y[3] = -5.0;
  waypoint_y[4] = 2.0;

  ros::init(argc, argv, "controller");
  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle n;
  ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/odom", 1000, callBack);
  ros::Rate loop_rate(rate);
  
  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;
    float diff_x = waypoint_x[current] - x_pos;
    float diff_y = waypoint_y[current] - y_pos;
    float pos = sqrt(pow(x_pos,2) + pow(y_pos,2));
    float abs_diff_x = fabs(diff_x);
    float abs_diff_y = fabs(diff_y);
    float diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
    float abs_diff = fabs(diff); 

    if(!(abs_diff_x < 0.05 && abs_diff_y < 0.05))
    {
      float ang_desired = atan2(diff_y , diff_x);

      float ang_diff = yaw - ang_desired;
      float abs_ang_diff = fabs(ang_diff);

      if (!(abs_ang_diff < 0.05))
      {
	if(ang_diff < 0) ang_diff += 2*M_PI;
	if(ang_diff > M_PI && ang_diff < 2*M_PI)
	{
	  ang_diff = 2*M_PI - ang_diff;

	  integral_ang_neg = 0;
	  integral_ang_pos += ang_diff * ki_lin / rate;
	  if(integral_ang_pos > max_output_ang)
	    integral_ang_pos = max_output_ang;

	  float derivative = (prior_ang_pos - ang_diff) * rate;

          msg.angular.z = kp_ang * ang_diff + integral_ang_pos + kd_ang * derivative;
	  if(msg.angular.z > max_output_ang)
	    msg.angular.z = max_output_ang;

	  prior_ang_pos = ang_diff;
	}
	else
	{
	  integral_ang_pos = 0;
	  integral_ang_neg += ang_diff * ki_lin * -1 / rate;
	  if(integral_ang_neg < min_output_ang)
	    integral_ang_neg = min_output_ang;

	  float derivative = (ang_diff - prior_ang_neg) * rate;

	  msg.angular.z = (kp_ang * ang_diff + -1 * integral_ang_neg + kd_ang * derivative) * -1;
	  if(msg.angular.z < min_output_ang) 
	    msg.angular.z = min_output_ang;

	  prior_ang_neg = ang_diff;
	}
      }
      else
      {
	integral_ang_pos = 0;
	integral_ang_neg = 0;

	integral_lin += abs_diff * ki_lin;
	if(integral_lin > max_output_lin)
	  integral_lin = max_output_lin;

	float derivative = (prior_lin - diff) * rate;

        msg.linear.x = kp_lin * abs_diff + integral_lin + kd_lin * derivative;
	if(msg.linear.x > max_output_lin)
	  msg.linear.x = max_output_lin;

	prior_lin = diff;
      }
      ROS_INFO("%f", msg.angular.z);
    }
    else if(current + 1 == waypoints)
    {
      ROS_INFO("Destination Reached");
    }
    else
    {
      current++;

      integral_lin = 0;
      integral_ang_pos = 0;
      integral_ang_neg = 0;

      prior_lin = 0;
      prior_ang_pos = 0;
      prior_ang_neg = 0;
    }

    control_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
