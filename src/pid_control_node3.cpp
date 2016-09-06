// This version creates a leash for the Roomba
// Ideal for many waypoints situated close to each other

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

#include <tf/tf.h>
#include <math.h>

// Parameters and initial values

// Message publishing rate: Hertz
int rate = 100;

// Array of waypoints: Meters, Meters
std::vector<float> waypoint_x;
std::vector<float> waypoint_y;
int waypoints;
int current_waypoint = 0;

bool correct_direction = false;

// Roomba position: Meters, Meters, Radians (-pi, pi)
float x_position = 0;
float y_position = 0;
float yaw = 0;

// PID constants for linear controller
float kp_lin = 0.6;
float ki_lin = 0.35;
float kd_lin = 0.1;

// PID constants for angular controller
float kp_ang = 2.2;
float ki_ang = 0.3;
float kd_ang = 0.2;

// Integrals from PID. Keeps a separate positive and negative integral to avoid large swings
float integral_lin = 0;
float integral_ang_pos = 0;
float integral_ang_neg = 0;

// Prior error for Derivative
float prior_lin = 0;
float prior_ang_pos = 0;
float prior_ang_neg = 0;

// Maximum Roomba speed:
// Linear: (-0.5, 0.5) Meters per second
// Angular: (-4.25, 4.25) Meters per second
float min_output_lin = -0.5;
float max_output_lin = 0.5;
float min_output_ang = -4.25;
float max_output_ang = 4.25;

// Assuming the Roomba travels at 2 rad/s and 0.5 m/s
float radius = 0.5;

// Gets position and direction from Odometry
void callBack(const nav_msgs::Odometry msg)
{
  x_position = msg.pose.pose.position.x;
  y_position = msg.pose.pose.position.y;

  // Conversion from Quaternion to Yaw
  tf::Pose pose;
  tf::poseMsgToTF(msg.pose.pose, pose);
  yaw = tf::getYaw(pose.getRotation());
}

void waypointGenerator()
{
  int count = 0;
  for(float i=0; i<9.0; i+=0.05)
  {
    waypoint_x.resize(count+1);
    waypoint_y.resize(count+1);
    waypoint_x[count] = i;
    waypoint_y[count] = sin(i);
    count++;
  }
  waypoints = count;
}

void waypointCondenser(float waypoint_xTemp, float waypoint_yTemp)
{
  
}

int main(int argc, char **argv)
{
  waypointGenerator();
  // Array of waypoints
/**
  waypoint_x[0] = 0.0;
  waypoint_x[1] = 0.2;
  waypoint_x[2] = 0.4;
  waypoint_x[3] = 0.55;
  waypoint_x[4] = 0.7;
  waypoint_x[5] = 0.8;
  waypoint_x[6] = 0.9;
  waypoint_x[7] = 0.95;
  waypoint_x[8] = 1.0;
  waypoint_x[9] = 1.03;

  waypoint_y[0] = 0.0;
  waypoint_y[1] = 0.05;
  waypoint_y[2] = 0.1;
  waypoint_y[3] = 0.2;
  waypoint_y[4] = 0.3;
  waypoint_y[5] = 0.45;
  waypoint_y[6] = 0.6;
  waypoint_y[7] = 0.8;
  waypoint_y[8] = 1.0;
  waypoint_y[9] = 1.25;
*/

  // Creates a node with a publisher, subscriber, and sets a rate
  // Publishes to topic: /cmd_vel
  //   Message type: geometry_msgs/Twist
  // Subscribes to topic: /odom
  //   Message type: nav_msgs/Odometry
  ros::init(argc, argv, "controller");
  ros::init(argc, argv, "odom_listener");
  ros::NodeHandle n;
  ros::Publisher control_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("/odom", 1000, callBack);
  ros::Rate loop_rate(rate);
  
  // Counter for total loops. Loop runs as long as ROS is working
  int count = 0;
  while (ros::ok())
  {
    // Initializes a message to be sent. 
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.angular.z = 0;

    // Calculates relative and total differences in distance as well as their absolute values
    float diff_x = waypoint_x[current_waypoint] - x_position;
    float diff_y = waypoint_y[current_waypoint] - y_position;
    float abs_diff_x = fabs(diff_x);
    float abs_diff_y = fabs(diff_y);
    float diff = sqrt(pow(diff_x,2) + pow(diff_y,2));
    float abs_diff = fabs(diff); 

    // If the Roomba is not at it's destination, PID loops will run
    if(!(diff < 0.03))
    {
      // Calculates desired angle, difference from current angle and its absolute value 
      float ang_desired = atan2(diff_y , diff_x);
      float ang_diff = yaw - ang_desired;
      float abs_ang_diff = fabs(ang_diff);

      float derivative;

      // Angular PID: only runs if there is an angular offset
      if (!(abs_ang_diff < 0.01))
      {
	// Normalizes angular difference for easier comparison
	if(ang_diff < 0) ang_diff += 2*M_PI;

	// Checks which direction Roomba should rotate depending on whichever is closer
	if(ang_diff > M_PI && ang_diff < 2*M_PI)
	{
	  // Normalizes angular difference for positive rotation 
	  ang_diff = 2*M_PI - ang_diff;

	  // Resets negative angular integral to avoid swings
	  integral_ang_neg = 0;

	  // Calculation and capping of the positive angular integral
	  integral_ang_pos += ang_diff * ki_lin / rate;
	  if(integral_ang_pos > max_output_ang)
	    integral_ang_pos = max_output_ang;

	  // Calculation of the derivative
	  derivative = (ang_diff - prior_ang_pos) * rate * kd_ang;

	  // Calculation and capping of angular velocity
          msg.angular.z = kp_ang * ang_diff + integral_ang_pos + derivative;
	  if(msg.angular.z > max_output_ang)
	    msg.angular.z = max_output_ang;

	  // Preparation for next derivative calculation
	  prior_ang_pos = ang_diff;
	}
	else
	{
	  // Does the same as above except in the opposite direction
	  integral_ang_pos = 0;
	  integral_ang_neg += ang_diff * ki_lin * -1 / rate;
	  if(integral_ang_neg < min_output_ang)
	    integral_ang_neg = min_output_ang;

	  derivative = (prior_ang_neg - ang_diff) * rate * kd_ang;

	  msg.angular.z = (kp_ang * ang_diff + -1 * integral_ang_neg + derivative) * -1;
	  if(msg.angular.z < min_output_ang) 
	    msg.angular.z = min_output_ang;

	  prior_ang_neg = ang_diff;
	}
        correct_direction = false;
      }
      else
      {
	// Resets integrals
      	integral_ang_pos = 0;
	integral_ang_neg = 0;
	correct_direction = true;
      }

      // Linear PID
      // Kicks in when the robot if at most 45 degrees off the correct direction
      // unless the robot is close to the destination, in which case the robot
      // must be in the correct direction to move
      float ang_diff_adapter = asin((diff/2) / radius);
      if(abs_ang_diff < ang_diff_adapter)
// && ((abs_diff > 0.1) || (abs_diff < 0.1 && correct_direction)))
      {
	// Same as angular PID except in linear direction
	// Since Roomba is slow, integral only starts building up at closer distances
	if(abs_diff < 0.3)
	{
	  integral_lin += abs_diff * ki_lin;
	  if(integral_lin > max_output_lin)
	    integral_lin = max_output_lin;
	}
	else
	  integral_lin = 0;

	float derivative = (diff - prior_lin) * rate;

        msg.linear.x = kp_lin * abs_diff + integral_lin + kd_lin * derivative;

	//FOR TESTING
	msg.linear.x = 0.25;

	if(msg.linear.x > max_output_lin)
  	  msg.linear.x = max_output_lin;

	prior_lin = diff;
      }
      else
	integral_lin = 0;
      // Prints out angular velocity for debugging and PID optimization
      //ROS_INFO("%f", msg.angular.z);
      ROS_INFO("%f", msg.linear.x);
    }
    // Destination reached if all waypoints are reached,
    // otherwise resets everything for next waypoint
    else if(current_waypoint + 1 == waypoints)
    {
      ROS_INFO("Destination Reached");
    }
    else
    {
      current_waypoint++;

      // integral_lin = 0;
      integral_ang_pos = 0;
      integral_ang_neg = 0;

      prior_lin = 0;
      prior_ang_pos = 0;
      prior_ang_neg = 0;
    }

    // Publishes message, increases ROS counter, then sleeps until next iteration
    control_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
