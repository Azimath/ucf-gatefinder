#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class SimpleNav
{
  ros::NodeHandle nh_;
  geometry_msgs::Vector3 offsetVector;
  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient ac;
  ros::Subscriber vector_sub;

public:
  SimpleNav() : ac("move_base", true)
  {
    vector_sub  = nh_.subscribe("/targetPos", 1, &SimpleNav::vectorCb, this);

  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move base action server to come up");
  }  
  
  }

  void vectorCb(const geometry_msgs::Vector3& msg)
  {
    float x = msg.x;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    if(x > 0){ // going right
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(-1);
    }
    if(x < 0){
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1);
    } // going left
    else{ // go forward
      goal.target_pose.pose.position.x = 0.5;
      goal.target_pose.pose.orientation.w = 1.0;
    }

    ROS_INFO("Sending Goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState()==actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward for some reason");


  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_drive");
  
  SimpleNav sn;
  ros::spin();


    return 0;
}
