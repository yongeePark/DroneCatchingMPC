#ifndef __controller_node__
#define __controller_node__
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <my_mpc/mympc.h>
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/State.h"
#include "geometry_msgs/PoseStamped.h"
#include "trajectory_msgs/MultiDOFJointTrajectory.h"
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

//#include <mav_msgs/RollPitchYawrateThrust.h>
// #include <mav_msgs/RollPitchYawrateThrust.h>
class ControllerNode
{
public:
    // Constructor
    ControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
    :nh_(nh), nh_param_(nh_param), is_armed_(false), is_offboard_(false), set_mass_(false)
    {
        std::cout<<"[controller_node.h] : Running controller node!"<<std::endl;
        pub_              = nh_.advertise<mavros_msgs::AttitudeTarget>("/scout/mavros/setpoint_raw/attitude", 100);
        pub_marker_       = nh_.advertise<visualization_msgs::Marker>("/scout/predicted_traj", 10);
        pub_direction_    = nh_.advertise<geometry_msgs::PoseStamped>("/scout/debug_pose", 10);

        sub_odom_         = nh_.subscribe("/scout/mavros/local_position/odom", 100, &ControllerNode::OdometryCallback, this);
        sub_state_        = nh_.subscribe("/scout/mavros/state", 1, &ControllerNode::StateCallback, this); // to control OFFBOARD and ARMING
        sub_goal_         = nh_.subscribe("/scout/goal_pose", 1, &ControllerNode::GoalCallbackByPoseStamped, this);
        sub_goal_mission_ = nh_.subscribe("/scout/goal_mission", 1, &ControllerNode::GoalCallbackByMission, this); // this is for debug
        sub_goalaction_   = nh_.subscribe("/scout/GoalAction",1,&ControllerNode::GoalActionCallback,this); //this is for connection with mission_planner
        sub_trajectory_   = nh_.subscribe("/scout/planning/trajectory",1,&ControllerNode::GoalCallbackByTrajectory,this); // trajectory
        
  

        controller_ = new mympc::ModelPredictiveController(nh, nh_param);
        
        attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE ;
        attitude_target.orientation.x = 0;
        attitude_target.orientation.y = 0;
        attitude_target.orientation.z = 0;
        attitude_target.orientation.w = 1;

        attitude_target.thrust = 0;

        if (!nh_param_.getParam("/scout/controller_node/thrust_to_throttle",thrust_to_throttle_))
        {
            std::cout<<"[Error] Please set [k_yaw] parameter"<<std::endl;
            thrust_to_throttle_ = 0.1;
        }

        set_mass_ = nh_param_.getParam("/scout/controller_node/mass",mass_);
        if (set_mass_ == false)
        {
            std::cout<<"[Error] Please set [mass] parameter"<<std::endl;
	        mass_ = 10000;// to make the drone can't fly!
        }
        if (!nh_param_.getParam("/scout/controller_node/throttle_limit",throttle_limit_))
        {
            std::cout<<"[Error] Please set [throttle_limit] parameter"<<std::endl;
            throttle_limit_ = 0.1;
        }
    }
    
    void OdometryCallback(const nav_msgs::Odometry::ConstPtr& msg); // Odometry Callback
    void StateCallback(const mavros_msgs::State::ConstPtr& msg);    // State Callback
    
    // It is recommended to use this callback.
    void GoalCallbackByTrajectory(const trajectory_msgs::MultiDOFJointTrajectory::ConstPtr& msg);
    // NEVER USE THESE TWO WHILE DOING REAL DRONE TEST!
    // IT WILL MAKE THE DRONE MOVE AT ITS MAXIMUM SPEED
    void GoalCallbackByMission(const std_msgs::Float32MultiArray& msg);
    void GoalCallbackByPoseStamped(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void PublishCommand();
    void GoalActionCallback(const std_msgs::Float32MultiArray& goalaciton);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_param_;
    ros::Publisher  pub_;
    ros::Publisher  pub_debug_;
    ros::Publisher  pub_marker_;
    ros::Publisher  pub_direction_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_state_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_goal_mission_;
    ros::Subscriber sub_goalaction_;
    ros::Subscriber sub_trajectory_;
    mympc::ModelPredictiveController* controller_;

    //command
    Eigen::Vector4d* ref_attitude_thrust = new Eigen::Vector4d;
    mavros_msgs::AttitudeTarget attitude_target;

    double thrust_to_throttle_;
    double mass_;
    double throttle_limit_;
    bool is_armed_;
    bool is_offboard_;
    //dangerous check
    bool set_mass_;
    
};

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
#endif
