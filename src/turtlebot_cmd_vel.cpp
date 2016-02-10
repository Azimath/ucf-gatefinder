#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

class TurtlebotCmdVel
{
  ros::NodeHandle nh_;
  // geometry_msgs::Twist moveCommand;
  sensor_msgs::LaserScan scanData;
  ros::Subscriber vector_sub;
  ros::Subscriber laser_sub;
  ros::Publisher move_pub;

public:
  TurtlebotCmdVel()
  {
    vector_sub = nh_.subscribe("/targetPos", 1, &TurtlebotCmdVel::vectorCb, this);
    laser_sub = nh_.subscribe("/scan", 1, &TurtlebotCmdVel::laserCb, this);
    move_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }
  /* Callback function for the /targetPos topic
     Attempts to move or turn the robot based on
     the vector received from image calculations
   */
  void vectorCb(const geometry_msgs::Vector3& msg)
  {
    geometry_msgs::Twist moveCommand;
    if(msg.x > 0.05){
      moveCommand.angular.x = 0.2;
    }if(msg.x < -0.05){
      moveCommand.angular.x = -0.2;
    }if(msg.x <= 0.05 && >= -0.05){
      moveCommand.linear.x = 0.2;
    }
    move_pub.publish(moveCommand); 
    ROS_INFO("MOVING");
  }

  /* Callback function for the /scan topic
     Attempts to stop the robot if the laser
     scanner detects an obstacle
   */
  void laserCb(const sensor_msgs::LaserScan& msg)
  {
    int numRanges = (int)(msg.angle_max - msg.angle_min) / msg.angle_increment;
    int minRange = 0.75;
    geometry_msgs::Twist moveCommand;
    for(int i = 0; i < numRanges; i++)
      {
	if(msg.ranges[i] < minRange){
	  moveCommand.linear.x = 0;
	  moveCommand.linear.y = 0;
	  moveCommand.linear.z = 0;
	  moveCommand.angular.x = 0;
	  moveCommand.angular.y = 0;
	  moveCommand.angular.z = 0;
	}
      }
    move_pub.publish(moveCommand);
  }
};

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "turtlebot_cmd_vel");
    TurtlebotCmdVel tbc;
    ros::spin();

    return 0;
  }
