#include <controller_node.h>


// callback
void ControllerNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run odometry callback!"<<std::endl;
    controller_->setOdometry(*msg);
    controller_->setState(is_armed_,is_offboard_);
    PublishCommand(); // calculate and publish
}
void ControllerNode::StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    if (msg->armed == true){is_armed_ = true;}
    else{is_armed_ = false;}
    if (msg->mode == "OFFBOARD"){is_offboard_ = true;}
    else{is_offboard_ = false; }
}
void ControllerNode::GoalCallbackByTrajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run goal callback!"<<std::endl;
    controller_->setTrajectory(*msg);
    // PublishCommand();
}
void ControllerNode::GoalCallbackByPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run goal callback!"<<std::endl;
    controller_->setGoal(*msg);
    // PublishCommand();
}

//this is for former mission_planner
void ControllerNode::GoalActionCallback(const std_msgs::Float32MultiArray& goalaction)
{
    double goal_x   = goalaction.data[1];
    double goal_y   = goalaction.data[2];
    double goal_z   = goalaction.data[3];
    double goal_yaw = goalaction.data[4]; // radian

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = goal_z; 

    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(goal_yaw, Eigen::Vector3d::UnitZ());
    
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    controller_->setGoal(goal);
}
void ControllerNode::GoalCallbackByMission(const std_msgs::Float32MultiArray& msg)
{
  // Mission consists of 4 messages.
  // x, y, z, yaw[deg]
    double goal_x   = msg.data[0];
    double goal_y   = msg.data[1];
    double goal_z   = msg.data[2];
    double goal_yaw = msg.data[3] / 180 * M_PI;
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = goal_z; 
    
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(goal_yaw, Eigen::Vector3d::UnitZ());
    
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();

    controller_->setGoal(goal);
}

// main function
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ControllerNode controller_node(nh, nh_param);
  ros::Rate loop_rate(30);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
