#include "px4_trajectories/px4_circular_trajectory.h"

#include <vector>

#include <autopilot/autopilot_states.h>
#include <polynomial_trajectories/polynomial_trajectory_settings.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/geometry_eigen_conversions.h>
#include <std_msgs/Bool.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>
#include <Eigen/Dense>

namespace Px4CircularTrajectory {

Px4CircularTrajectory::Px4CircularTrajectory()
    : executing_trajectory_(false),
      sum_position_error_squared_(0.0),
      max_position_error_(0.0),
      sum_thrust_direction_error_squared_(0.0),
      max_thrust_direction_error_(0.0) {
  ros::NodeHandle nh;

  arm_pub_ = nh.advertise<std_msgs::Bool>("bridge/arm", 1);

  measure_tracking_timer_ =
      nh_.createTimer(ros::Duration(1.0 / kExecLoopRate_),
                      &Px4CircularTrajectory::measureTracking, this);
}

Px4CircularTrajectory::~Px4CircularTrajectory() {}

void Px4CircularTrajectory::measureTracking(const ros::TimerEvent& time) {
  if (executing_trajectory_) {
    // Position error
    const double position_error =
        autopilot_helper_.getCurrentPositionError().norm();
    sum_position_error_squared_ += pow(position_error, 2.0);
    if (position_error > max_position_error_) {
      max_position_error_ = position_error;
    }

    // Thrust direction error
    const Eigen::Vector3d ref_thrust_direction =
        autopilot_helper_.getCurrentReferenceOrientation() *
        Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d thrust_direction =
        autopilot_helper_.getCurrentOrientationEstimate() *
        Eigen::Vector3d::UnitZ();

    const double thrust_direction_error =
        acos(ref_thrust_direction.dot(thrust_direction));
    sum_thrust_direction_error_squared_ += pow(thrust_direction_error, 2.0);
    if (thrust_direction_error > max_thrust_direction_error_) {
      max_thrust_direction_error_ = thrust_direction_error;
    }
  }
}

void Px4CircularTrajectory::run() {
  ros::Rate command_rate(kExecLoopRate_);

  ros::Duration(3.0).sleep();

  ///////////////
  // Check sending reference states
  ///////////////

  // Generate trajectory, sample it and send it as reference states
  const double max_vel = 3;
  const double max_thrust = 15.0;
  const double max_roll_pitch_rate = 0.5;
  const Eigen::Vector3d center = Eigen::Vector3d(-2.0, 0.0, 1.0);
  const Eigen::Vector3d position_cmd = Eigen::Vector3d(0, 0.0, 1.0);

  quadrotor_common::TrajectoryPoint start_state;
  //start_state.position = autopilot_helper_.getCurrentPositionEstimate();
  start_state.position = position_cmd;
  start_state.heading = 0.0;
  quadrotor_common::TrajectoryPoint end_state;
  end_state.position = Eigen::Vector3d(1.5, 1.7, 1.2);
  end_state.heading = M_PI;

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          center, 2.0, max_vel, 0.0, 10*M_PI, kExecLoopRate_);

  //trajectory_generation_helper::heading::addConstantHeadingRate(
    //  start_state.heading, end_state.heading, &manual_traj);

  bool autopilot_was_in_reference_control_mode = false;

  while (ros::ok() && !manual_traj.points.empty()) {
    autopilot_helper_.sendReferenceState(manual_traj.points.front());
    manual_traj.points.pop_front();
    ros::spinOnce();
    if (!autopilot_was_in_reference_control_mode &&
        autopilot_helper_.getCurrentAutopilotState() ==
            autopilot::States::REFERENCE_CONTROL) {
      autopilot_was_in_reference_control_mode = true;
    }
    command_rate.sleep();
  }

  ///////////////
  // Check trajectory control
  ///////////////

  // Generate trajectories and send them as complete trajectories
  // One polynomial to enter a ring and a ring to check execution of
  // consecutive trajectories

  // Ring trajectory with enter segment
      /*
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(-0.5, 0.0, 1.5));
  way_points.push_back(Eigen::Vector3d(1.5, -1.5, 0.6));
  way_points.push_back(Eigen::Vector3d(3.5, 0.0, 2.0));
  way_points.push_back(Eigen::Vector3d(1.5, 2.0, 0.6));

  Eigen::VectorXd initial_ring_segment_times =
      Eigen::VectorXd::Ones(int(way_points.size()));
  polynomial_trajectories::PolynomialTrajectorySettings
      ring_trajectory_settings;
  ring_trajectory_settings.continuity_order = 4;
  Eigen::VectorXd minimization_weights(5);
  minimization_weights << 0.0, 1.0, 1.0, 1.0, 1.0;
  ring_trajectory_settings.minimization_weights = minimization_weights;
  ring_trajectory_settings.polynomial_order = 11;
  ring_trajectory_settings.way_points = way_points;

  quadrotor_common::Trajectory ring_traj = trajectory_generation_helper::
      polynomials::generateMinimumSnapRingTrajectoryWithSegmentRefinement(
          initial_ring_segment_times, ring_trajectory_settings, max_vel,
          max_thrust, max_roll_pitch_rate, kExecLoopRate_);

  polynomial_trajectories::PolynomialTrajectorySettings
      enter_trajectory_settings = ring_trajectory_settings;
  enter_trajectory_settings.way_points.clear();

  quadrotor_common::Trajectory enter_traj =
      trajectory_generation_helper::polynomials::generateMinimumSnapTrajectory(
          Eigen::VectorXd::Ones(1), end_state, ring_traj.points.front(),
          enter_trajectory_settings, 1.03 * max_vel, 1.03 * max_thrust,
          max_roll_pitch_rate, kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(
      end_state.heading, 0.0, &enter_traj);
  trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                &ring_traj);

  autopilot_helper_.sendTrajectory(enter_traj);
  autopilot_helper_.sendTrajectory(ring_traj); */

    }  // namespace rpg_quadrotor_integration_test
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "px4_circular_trajectory");

  Px4CircularTrajectory::Px4CircularTrajectory myTraj;
  myTraj.run();

  return 0;
}
