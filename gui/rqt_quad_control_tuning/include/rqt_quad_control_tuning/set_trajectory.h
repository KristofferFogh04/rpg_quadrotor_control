#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <dynamic_reconfigure/server.h>
#include <rqt_quad_control_tuning/setControlParametersConfig.h>
#include <rqt_quad_control_tuning/setTrajectoryConfig.h>
#include <autopilot/autopilot_helper.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <cmath>
#include <random>


class SetTrajectory
{

 public:

  SetTrajectory(ros::NodeHandle& nh_);

  void trajectoryCallback(
    rqt_quad_control_tuning::setTrajectoryConfig &config,
    uint32_t level);

  void computeCircularTrajectory(double max_vel, double radius, int num_rotations);
  void computeVerticalCircularTrajectory(
    double max_vel, 
    double radius, 
    int num_rotations, 
    double orientation);
  void computeMinimumSnapRingTrajectory(
    double max_vel, 
    double max_thrust, 
    double max_roll_pitch_rate);
  void computeLemniscateTrajectory(
    const double radius, const double speed,
    const double phi_start, const double phi_end,
    const double sampling_frequency);
  void computeSquareTrajectory(
    double radius, 
    double speed, 
    double max_thrust, 
    double max_roll_pitch_rate);
  void computeRandomMinSnap();
  double RandomDouble(double a, double b);


 private:

  ros::NodeHandle nh_; 
  ros::Publisher trajectoryPublisher_;

  autopilot_helper::AutoPilotHelper autopilot_helper_;
  quadrotor_common::Trajectory target_trajectory_;

  int traj_type_;
  bool exitFlag;

  // Constants
  static constexpr double kExecLoopRate_ = 30.0;

};  // End of Class
