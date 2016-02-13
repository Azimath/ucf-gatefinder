#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>

float Clamp(float val, float min, float max);

class TurtlebotCmdVel
{
  const float maxTurnRate = 1.0;
  const float minTurnRate = -1.0;
  const float maxDriveSpeed = 0.3;
  const float minDriveSpeed = 0.0;
  ros::NodeHandle nh_;
  // geometry_msgs::Twist moveCommand;
  sensor_msgs::LaserScan scanData;
  ros::Subscriber vector_sub;
  ros::Subscriber laser_sub;
  ros::Publisher move_pub;
  
  float prevAngularZ;
  float currAngularZ;
  float prevLinearX;
  float currLinearX;
  
public:
  TurtlebotCmdVel():prevAngularZ(0),
		    currAngularZ(0),
		    prevLinearX(0),
		    currLinearX(0)
  {
    vector_sub = nh_.subscribe("/targetPos", 1, &TurtlebotCmdVel::vectorCb, this);
 //   laser_sub = nh_.subscribe("/scan", 1, &TurtlebotCmdVel::laserCb, this);
    move_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1000);
  }
  /* Callback function for the /targetPos topic
     Attempts to move or turn the robot based on
     the vector received from image calculations
   */
  void vectorCb(const geometry_msgs::Vector3& msg)
  {
    geometry_msgs::Twist moveCommand;
    prevAngularZ = currAngularZ;
    prevLinearX = currLinearX;
    if(msg.x > 30){
      currAngularZ = prevAngularZ - 0.2;
      currLinearX = prevLinearX - 0.1;
      ROS_INFO("TURNING LEFT");
    }if(msg.x < -30){
      currAngularZ = prevAngularZ + 0.2;
      currLinearX = prevLinearX - 0.1;
      ROS_INFO("TURNING RIGHT");
    }if(msg.x < 30 && msg.x > 15){
      currAngularZ = prevAngularZ - 0.1;
      currLinearX = prevLinearX - 0.05;
      ROS_INFO("TURNING LEFT and SLOWING");
    }if(msg.x > -30 && msg.x < -15){
      currAngularZ = prevAngularZ + 0.1;
      currLinearX = prevLinearX -0.05;
      ROS_INFO("TURNING RIGHT and SLOWING");
    }if(msg.x <= 15 && msg.x >= -15){
      currAngularZ = prevAngularZ / 2;
      currLinearX = prevLinearX + 0.1;
      ROS_INFO("SLOWING TURN and MOVING FORWARD");
    }
    moveCommand.angular.z = Clamp(currAngularZ, minTurnRate, maxTurnRate);
    moveCommand.linear.x = Clamp(currLinearX, minMoveSpeed, maxMoveSpeed);
    move_pub.publish(moveCommand);
  }

  /* Callback function for the /scan topic
     Attempts to stop the robot if the laser
     scanner detects an obstacle
   */
/*  void laserCb(const sensor_msgs::LaserScan& msg)
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
	  ROS_INFO("STOPPING");
	}
      }
    move_pub.publish(moveCommand);
  }
*/
};

float Clamp(float val, float min, float max)
{
  if(val > max)
    val = max;
  if(val < min)
    val = min;
  return val;
}

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "turtlebot_cmd_vel");
    TurtlebotCmdVel tbc;
    ros::spin();

    return 0;
  }
