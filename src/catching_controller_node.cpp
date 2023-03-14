#include <catching_controller_node.h>


// Current Odometry callback
void ControllerNode::OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run odometry callback!"<<std::endl;
    pose_[0] = msg->pose.pose.position.x;
    pose_[1] = msg->pose.pose.position.y;
    pose_[2] = msg->pose.pose.position.z;
    controller_->setOdometry(*msg);
    controller_->setState(is_armed_,is_offboard_);
    PublishCommand(); // calculate and publish
}
// Current State Callback
// arming, offboard
void ControllerNode::StateCallback(const mavros_msgs::State::ConstPtr& msg)
{
    if (msg->armed == true){is_armed_ = true;}
    else{is_armed_ = false;}
    if (msg->mode == "OFFBOARD"){is_offboard_ = true;}
    else{is_offboard_ = false; }
}

//trajectory Callback
void ControllerNode::GoalCallbackByTrajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run goal callback!"<<std::endl;
    controller_->setTrajectory(*msg);
    // PublishCommand();
}

void ControllerNode::TargetPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // std::cout<<"[controller_node.h] : run goal callback!"<<std::endl;
    geometry_msgs::PoseStamped posemsg;
    posemsg.pose.position.x = msg->pose.pose.position.x;
    posemsg.pose.position.y = msg->pose.pose.position.y;
    posemsg.pose.position.z = msg->pose.pose.position.z;
    if(is_takeoff_)
    {
        controller_->setGoal(posemsg);
    }
    // controller_->setGoal(*msg);
    // PublishCommand();
}
void ControllerNode::PublishCommand()
{
    // std::cout<<"publish command"<<std::endl;
    static int publish_count = 0;
    
    controller_->calculateRollPitchYawrateThrustCommand(ref_attitude_thrust);
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd((*ref_attitude_thrust)[0], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd((*ref_attitude_thrust)[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd((*ref_attitude_thrust)[2], Eigen::Vector3d::UnitZ());
    
    tf::Quaternion quat_from_rpy;
    quat_from_rpy.setRPY((*ref_attitude_thrust)[0], (*ref_attitude_thrust)[1],(*ref_attitude_thrust)[2]);

    attitude_target.orientation.x = quat_from_rpy.x();
    attitude_target.orientation.y = quat_from_rpy.y();
    attitude_target.orientation.z = quat_from_rpy.z();
    attitude_target.orientation.w = quat_from_rpy.w();

    attitude_target.body_rate.x = 0;
    attitude_target.body_rate.y = 0;
    attitude_target.body_rate.z = 0;

    double throttle = (*ref_attitude_thrust)[3] / mass_ / 9.8 * thrust_to_throttle_; // 2d_platform
    // double throttle = (*ref_attitude_thrust)[3] / 2.30 / 9.8 * 0.66; // gazebo
    
    // if (throttle >= 0.57)   {throttle = 0.57;}
    // attitude_target.thrust = throttle;

    attitude_target.thrust = throttle >= throttle_limit_ ? throttle_limit_ : throttle; 
    cout<<"throttle : "<<throttle;
    // PUBLISH!!!
    if(set_mass_)   {pub_.publish(attitude_target);}
   
    publish_count++;
    if(publish_count > 10)
    {
        // std::cout<<"=======[controller_node.h]============="<<std::endl;
        // std::cout<<"command"<<std::endl;
        // std::cout<<
        // "roll : "<<euler_angles(0) * 180 / M_PI<<std::endl<<
        // "pitch : "<<euler_angles(1) * 180 / M_PI<<std::endl<<
        // "yaw : "<<euler_angles(2) * 180 / M_PI<<std::endl;


        //std::cout<<"==========================================="<<std::endl;
        // std::cout<<"thrust   : "<<(*ref_attitude_thrust)[3]<<std::endl
        //          <<"acc   : "<<(*ref_attitude_thrust)[3] / 1.5 <<std::endl
        //          <<"throttle : "<<throttle<<std::endl;
        publish_count=0;
        
    }

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.pose.orientation.w =1.0;

    marker.scale.x = 0.05;  marker.scale.y = 0.05;
    marker.color.g = 1.0f;  marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::POINTS;


    // marker.points.
    controller_->getPredictedState(marker);
    pub_marker_.publish(marker);

}

void ControllerNode::GetPose(double pose[])
{
    pose[0] = pose_[0];
    pose[1] = pose_[1];
    pose[2] = pose_[2];
}
bool ControllerNode::GetIsTakeoff()
{
    return is_takeoff_;
}
void ControllerNode::SetIsTakeoff(bool cmd)
{
    is_takeoff_ = cmd;
}
void ControllerNode::Takeoff(double height)
{
    geometry_msgs::PoseStamped goal;

    goal.pose.position.x = 0;
    goal.pose.position.y = 0;
    goal.pose.position.z = 1;

    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = 0;
    goal.pose.orientation.w = 1;

    controller_->setGoal(goal);
}



// main function
int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "catching_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_param("~");
  ControllerNode controller_node(nh, nh_param);
  ros::Rate loop_rate(30);

  double takeoff_height = 2;

  double current_pose[3];
  static int print_count = 0;
  while(ros::ok())
  {
    // if takeoff is not done
    if(!controller_node.GetIsTakeoff())
    {
        // publish takeoff message
        controller_node.Takeoff(takeoff_height);
        controller_node.GetPose(current_pose);
        if(current_pose[2] > 1.0)
        {
            controller_node.SetIsTakeoff(true);
        }
    }
    else
    {
        //
    }


    print_count++;
    if(print_count>10)
    {
        print_count = 0;
        cout<<"============================="<<endl
        <<"catching_controller_node"<<endl
        <<"Takeoff : "<<controller_node.GetIsTakeoff()<<endl;
    }


    //if takeoff is
    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
