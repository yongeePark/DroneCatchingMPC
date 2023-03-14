#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// main function

using namespace std;
bool g_is_takeoff = false;
bool g_is_armed = false;
bool g_is_offboard = false;
typedef geometry_msgs::PoseStamped pose;

pose g_current_pose;
void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    g_current_pose = *msg;
    if(msg->pose.position.z > 1.0)
    {
        g_is_takeoff = true;
    }
}
void callback_state(const mavros_msgs::State::ConstPtr& msg)
{
    if(msg->mode == "OFFBOARD")
    {
        g_is_offboard = true;
    }
    else
    {
        g_is_offboard = false;
    }

    g_is_armed = msg->armed ? true : false;

}


int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "target_control_node");
    ros::NodeHandle nh;

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local",10);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/uav2/mavros/setpoint_velocity/cmd_vel",10);
    ros::Subscriber position_sub = nh.subscribe("/uav2/mavros/local_position/pose", 1, CallbackPose);
    ros::Subscriber state_sub = nh.subscribe ("/uav2/mavros/state" , 1, &callback_state);
    ros::Rate loop_rate(30);

    ros::ServiceClient  arming_client    = nh.serviceClient<mavros_msgs::CommandBool> ("/uav2/mavros/cmd/arming");
    ros::ServiceClient  set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>     ("/uav2/mavros/set_mode");


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;


    double target_height = 3;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = target_height;

    geometry_msgs::TwistStamped vel_msg;
    vel_msg.header.stamp = ros::Time::now();
    vel_msg.header.frame_id = "map";
    vel_msg.twist.linear.x = 0.4;
    vel_msg.twist.linear.y = 0;
    vel_msg.twist.linear.z = 0;

    
    //takeoff to 0,0,1
    // and move based on the velocity message

    
    static int print_count = 0;
    while(ros::ok())
    {
        // arm, offboard check

        // if it is not offboard
        if( !g_is_offboard )
        {
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                // ROS_INFO("Offboard enabled");
            }
        }
        if ( !g_is_armed )
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                // ROS_INFO("Vehicle armed");
            }

        }

        // 
        
        if(!g_is_takeoff)
        {
            pose_pub.publish(msg);
        }
        else
        {   
            vel_msg.header.stamp = ros::Time::now();
            vel_msg.twist.linear.z = target_height - g_current_pose.pose.position.z;
            vel_pub.publish(vel_msg);
        }

        print_count++;
        if(print_count > 10)
        {
            print_count = 0;
            cout<<"====================="<<endl
            <<"Target Controller Node"<<endl
            <<"is takeoff : "<<g_is_takeoff<<endl
            <<"is armed : "<<g_is_armed<<endl
            <<"is offboard : "<<g_is_offboard<<endl;
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
