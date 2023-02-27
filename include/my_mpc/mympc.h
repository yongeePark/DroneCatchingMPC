#ifndef __my_mpc__
#define __my_mpc__

# include <iostream>
#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/LU> // for .inverse() function
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <lapacke.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <tf/transform_datatypes.h>

#include <visualization_msgs/Marker.h>
// ACADO SOLVER
using namespace std;

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define NX          ACADO_NX	/* number of differential states */
#define NXA         ACADO_NXA	/* number of algebraic states */
#define NU          ACADO_NU	/* number of control inputs */
#define N          	ACADO_N		/* number of control intervals */
#define NY			ACADO_NY	/* number of measurements, nodes 0..N-1 */
#define NYN			ACADO_NYN	/* number of measurements, node N */
#define NUM_STEPS   100		/* number of simulation steps */
#define VERBOSE     1			/* show iterations: 1, silent: 0  */

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace mympc{

lapack_logical select_lhp(const double *real, const double *imag)
{
  return *real < 0.0;
}
void adjust_angle(double& angle, double limit_angle)
{
  // for roll and pitch, if it is bigger than pi.
  // for yaw, if it is bigger than 2 * pi 
  while(abs(angle) > limit_angle)
  {
    if (angle>0) { angle = angle - 2 * M_PI;}
    else { angle = angle + 2 * M_PI;}
  }
}
class ModelPredictiveController
{
public:
  //ControllerNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
  ModelPredictiveController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param);
  ~ModelPredictiveController();

  bool setControllerParameters();
  Eigen::MatrixXd solveCARE(Eigen::MatrixXd Q, Eigen::MatrixXd R);

  void initializeAcadoSolver(Eigen::VectorXd x0);

  void setOdometry(const nav_msgs::Odometry& odometry);
  void setState(bool is_armed, bool is_offboard);
  void setGoal(const geometry_msgs::PoseStamped& pose);
  void setTrajectory(const trajectory_msgs::MultiDOFJointTrajectory& trajectory);
  void setPointsFromTrajectory();

  void calculateRollPitchYawrateThrustCommand(Eigen::Vector4d* ref_attitude_thrust);
  void getPredictedState(visualization_msgs::Marker& marker);
  
private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_param_;

  static constexpr double kGravity = 9.8066;

  bool initialized_parameters_;

  //MPC coefficients
  Eigen::Vector3d q_position_;
  Eigen::Vector3d q_velocity_;
  Eigen::Vector2d q_attitude_; // roll, pitch
  Eigen::Vector3d r_command_;

  double mass_;
  double roll_time_constant_;
  double roll_gain_;
  double pitch_time_constant_;
  double pitch_gain_;
  Eigen::Vector3d drag_coefficients_;
  
  
  double roll_limit_;
  double pitch_limit_;
  double yaw_rate_limit_;
  double thrust_min_;
  double thrust_max_;

  // solver matrices
  Eigen::Matrix<double, ACADO_NY, ACADO_NY> W_;
  Eigen::Matrix<double, ACADO_NYN, ACADO_NYN> WN_;
  Eigen::Matrix<double, ACADO_N + 1, ACADO_NX> state_;
  Eigen::Matrix<double, ACADO_N, ACADO_NU> input_;
  Eigen::Matrix<double, ACADO_N, ACADO_NY> reference_;
  Eigen::Matrix<double, 1, ACADO_NYN> referenceN_;
  Eigen::Matrix<double, ACADO_N + 1, ACADO_NOD> acado_online_data_;

  Eigen::Vector3d position_ref_, velocity_ref_;
  double yaw_ref_;
  trajectory_msgs::MultiDOFJointTrajectory trajectory_ref_;
  int trajectory_ref_index_;
  // odometry
  nav_msgs::Odometry odometry_;
  double current_yaw_;
  double k_yaw_;

  bool received_first_odometry_;
  bool received_trajectory_;
  bool finished_trajectory_;
  bool is_ready_;
};

}

#endif