#ifndef __controller_node__
#define __controller_node__
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <iostream>
#include <catching_mpc.h>
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
    :nh_(nh), nh_param_(nh_param), is_armed_(false), is_offboard_(false), is_takeoff_(false), set_mass_(false)
    {
        std::cout<<"[catching_controller_node.h] : Running controller node!"<<std::endl;
        pub_              = nh_.advertise<mavros_msgs::AttitudeTarget>("/uav1/mavros/setpoint_raw/attitude", 100);
        pub_marker_       = nh_.advertise<visualization_msgs::Marker>("/scout/predicted_traj", 10);
        pub_direction_    = nh_.advertise<geometry_msgs::PoseStamped>("/scout/debug_pose", 10);

        sub_odom_         = nh_.subscribe("/uav1/mavros/local_position/odom", 100, &ControllerNode::OdometryCallback, this);
        sub_state_        = nh_.subscribe("/scout/mavros/state", 1, &ControllerNode::StateCallback, this); // to control OFFBOARD and ARMING

        sub_goal_         = nh_.subscribe("/uav2/odom", 1, &ControllerNode::TargetPoseCallback, this); // target position, gazebo
        // sub_goal_         = nh_.subscribe("/uav2/mavros/setpoint_position/local", 1, &ControllerNode::TargetPoseCallback, this); // target position, 

        // sub_goal_mission_ = nh_.subscribe("/scout/goal_mission", 1, &ControllerNode::GoalCallbackByMission, this); // this is for debug
        // sub_goalaction_   = nh_.subscribe("/scout/GoalAction",1,&ControllerNode::GoalActionCallback,this); //this is for connection with mission_planner
        sub_trajectory_   = nh_.subscribe("/scout/planning/trajectory",1,&ControllerNode::GoalCallbackByTrajectory,this); // trajectory
        
  

        controller_ = new catching_mpc::ModelPredictiveController(nh, nh_param);
        
        attitude_target.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE | 
                                    mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
                                    mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE ;
        attitude_target.orientation.x = 0;
        attitude_target.orientation.y = 0;
        attitude_target.orientation.z = 0;
        attitude_target.orientation.w = 1;

        attitude_target.thrust = 0;

        if (!nh_param_.getParam("/scout/catching_mpc_controller/thrust_to_throttle",thrust_to_throttle_))
        {
            std::cout<<"[Error] Please set [k_yaw] parameter"<<std::endl;
            thrust_to_throttle_ = 0.1;
        }

        set_mass_ = nh_param_.getParam("/scout/catching_mpc_controller/mass",mass_);
        if (set_mass_ == false)
        {
            std::cout<<"[Error] Please set [mass] parameter"<<std::endl;
	        mass_ = 10000;// to make the drone can't fly!
        }
        if (!nh_param_.getParam("/scout/catching_mpc_controller/throttle_limit",throttle_limit_))
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

    void TargetPoseCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void PublishCommand();

    void GetPose(double []);
    bool GetIsTakeoff();
    void SetIsTakeoff(bool cmd);
    void Takeoff(double height);


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
    catching_mpc::ModelPredictiveController* controller_;

    double pose_[3];

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

    bool is_takeoff_;

    
    
};


#endif
