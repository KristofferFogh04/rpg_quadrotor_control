#include <rqt_quad_control_tuning/set_trajectory.h>

SetTrajectory::SetTrajectory(ros::NodeHandle& nh_) : // default constructor for
nh_(nh_)
{
  std::cout << "Void constructor" << std::endl;
  exitFlag = false;
  // ROS publishers:
  
}

void SetTrajectory::trajectoryCallback(
  rqt_quad_control_tuning::setTrajectoryConfig &config,
  uint32_t level){

  traj_type_ = config.trajectory;

  switch(traj_type_) {
  case 0:
    std::cout << "Doing nothing" << std::endl;
    break;
  case 1:
    std::cout << "Horizontal Circle" << std::endl;
    SetTrajectory::computeCircularTrajectory(
      config.speed, 
      config.circle_radius, 
      config.num_rotations);
    break;
  case 2:
    std::cout << "Vertical Circle" << std::endl;
    SetTrajectory::computeVerticalCircularTrajectory(
      config.speed, 
      config.circle_radius, 
      config.num_rotations, 
      config.circle_orientation);
    break;
  case 3:
    std::cout << "Minimum Snap Ring Trajectory" << std::endl;
    SetTrajectory::computeMinimumSnapRingTrajectory(
      config.speed, 
      config.max_thrust, 
      config.max_roll_pitch_rate);
    break;
  case 4:
    std::cout << "Lemniscate Trajectory" << std::endl;
    SetTrajectory::SetTrajectory::computeLemniscateTrajectory(
    config.circle_radius, 
    config.speed,
    0, config.num_rotations*2*M_PI,
    kExecLoopRate_);
    break;
  case 5:
    std::cout << "Square Minimum Snap" << std::endl;
    SetTrajectory::computeSquareTrajectory(      
      config.circle_radius,
      config.speed, 
      config.max_thrust, 
      config.max_roll_pitch_rate);
    break;
  case 6:
    std::cout << "Continuous Random Minimum Snap" << std::endl;
    SetTrajectory::computeRandomMinSnap();
    break;
  case 7:
    std::cout << "Exit Continuous Random Minimum Snap" << std::endl;
    exitFlag = true;
    break;
  default:
    std::cout << "Not recognized. Doing nothing" << std::endl;
    // code block
  }
}

void SetTrajectory::computeCircularTrajectory(
  double max_vel, 
  double radius, 
  int num_rotations){

  ros::Rate command_rate(kExecLoopRate_);

  // Generate trajectory, sample it and send it as reference states
  const Eigen::Vector3d center = Eigen::Vector3d(-1.0, 0.0, 1.5);

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          center, radius, max_vel, 0.0, num_rotations*2*M_PI, kExecLoopRate_);

  // First send go to command to get the drone to the start point
  // Send pose command
  autopilot_helper_.sendPoseCommand(manual_traj.points.front().position, 0.0);
  manual_traj.points.pop_front();

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  // Send entire trajectory
  autopilot_helper_.sendTrajectory(manual_traj);
}

void SetTrajectory::computeVerticalCircularTrajectory(
    double max_vel, 
    double radius, 
    int num_rotations, 
    double orientation){

  ros::Rate command_rate(kExecLoopRate_);

  // Generate trajectory, sample it and send it as reference states
  const Eigen::Vector3d center = Eigen::Vector3d(-2.0, 0.0, 1.5);

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          center, orientation, radius, max_vel, 0.0, -num_rotations*2*M_PI, kExecLoopRate_);

  // First send go to command to get the drone to the start point
  // Send pose command
  autopilot_helper_.sendPoseCommand(manual_traj.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  // Send entire trajectory
  autopilot_helper_.sendTrajectory(manual_traj);
}

void SetTrajectory::computeMinimumSnapRingTrajectory(
    double max_vel,
    double max_thrust,
    double max_roll_pitch_rate){
  
// Ring trajectory
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(-0.5, 0.0, 2.5));
  way_points.push_back(Eigen::Vector3d(1.5, -1.5, 1.6));
  way_points.push_back(Eigen::Vector3d(3.5, 0.0, 2.0));
  way_points.push_back(Eigen::Vector3d(1.5, 2.0, 1.6));

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

  trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                &ring_traj);

  // Last trajectory point is for some reason weird so we remove it
  ring_traj.points.pop_back();

  // Go to first point of trajectory manually
  autopilot_helper_.sendPoseCommand(ring_traj.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  // Send entire trajectory
  autopilot_helper_.sendTrajectory(ring_traj);

}

void SetTrajectory::computeLemniscateTrajectory(
    const double radius, const double speed,
    const double phi_start, const double phi_end,
    const double sampling_frequency) {

  quadrotor_common::Trajectory trajectory;
  trajectory.trajectory_type =
      quadrotor_common::Trajectory::TrajectoryType::GENERAL;

  const double phi_total = phi_end - phi_start;
  const double direction = phi_total / fabs(phi_total);
  const double omega = direction * fabs(speed / radius);
  const double angle_step = fabs(omega / sampling_frequency);

  for (double d_phi = 0.0; d_phi < fabs(phi_total); d_phi += angle_step) {
    const double phi = phi_start + direction * d_phi;
    const double sin_phi = sin(phi);
    const double sincos_phi = sin(phi)*cos(phi);

    quadrotor_common::TrajectoryPoint point;
    point.time_from_start = ros::Duration(fabs(d_phi / omega));
    point.position = radius * Eigen::Vector3d(cos(phi), sin(phi)*cos(phi), 1.5/radius);

    // All the derivatives
    point.velocity = 
        radius * omega * Eigen::Vector3d(-sin(phi), pow(cos(phi), 2.0) - pow(sin(phi), 2.0), 0.0);
    point.acceleration =
        radius * pow(omega, 2.0) * Eigen::Vector3d(-cos(phi), -4*cos(phi)*sin(phi) , 0.0);
    point.jerk =
        radius * pow(omega, 3.0) * Eigen::Vector3d(sin(phi), 4*(pow(sin(phi), 2.0) - pow(cos(phi), 2.0)), 0.0);
    point.snap =
        radius * pow(omega, 4.0) * Eigen::Vector3d(cos(phi), 16*cos(phi)*sin(phi), 0.0);

    trajectory.points.push_back(point);
  }

  // Go to first point of trajectory manually
  autopilot_helper_.sendPoseCommand(trajectory.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  // Send entire trajectories 
  autopilot_helper_.sendTrajectory(trajectory);

}

void SetTrajectory::computeSquareTrajectory(
  double radius, 
  double speed, 
  double max_thrust, 
  double max_roll_pitch_rate){

// Ring trajectory
  std::vector<Eigen::Vector3d> way_points;
  way_points.push_back(Eigen::Vector3d(radius, 0.0, 2.0));
  way_points.push_back(Eigen::Vector3d(0.0, radius, 2.0));
  way_points.push_back(Eigen::Vector3d(-radius, 0.0, 2.0));
  way_points.push_back(Eigen::Vector3d(0.0, -radius, 2.0));

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
          initial_ring_segment_times, ring_trajectory_settings, speed,
          max_thrust, max_roll_pitch_rate, kExecLoopRate_);

  trajectory_generation_helper::heading::addConstantHeadingRate(0.0, M_PI,
                                                                &ring_traj);

   // Last trajectory point is for some reason weird so we remove it
  ring_traj.points.pop_back();

  // Go to first point of trajectory manually
  autopilot_helper_.sendPoseCommand(ring_traj.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  // Send entire trajectory
  autopilot_helper_.sendTrajectory(ring_traj);

}

void SetTrajectory::computeRandomMinSnap(){

  double max_x = 2;
  double min_x = -2;
  double max_y = 2;
  double min_y = -2;
  double max_z = 2.5;
  double min_z = 0.2;
  int num_waypoints = 4;

  while (exitFlag == false){

    std::vector<Eigen::Vector3d> way_points;

    for (int i = 0; i < num_waypoints; i++) {
      way_points.push_back(Eigen::Vector3d(RandomDouble(min_x, max_x),
                                           RandomDouble(min_y, max_y),
                                           RandomDouble(min_z, max_z)));
    }
    //double num_rotations = RandomDouble(0, 25);
    //double velocity = RandomDouble(0.1, 4);
    //double max_thrust = RandomDouble(1, 30);
    //double max_roll_pitch_rate = RandomDouble(0, 2);
    double num_rotations = 3.14;
    double velocity = 1.5;
    double max_thrust = 15;
    double max_roll_pitch_rate = 1;

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
              initial_ring_segment_times, ring_trajectory_settings, velocity,
              max_thrust, max_roll_pitch_rate, kExecLoopRate_);

      trajectory_generation_helper::heading::addConstantHeadingRate(0.0, num_rotations,
                                                                    &ring_traj);

       // Last trajectory point is for some reason weird so we remove it
      ring_traj.points.pop_back();
      std::cout << "Waiting for hover" << std::endl;
      autopilot_helper_.waitForSpecificAutopilotState(
          autopilot::States::HOVER, 100.0, kExecLoopRate_);

      // Go to first point of trajectory manually
      std::cout << "Sending first pose command" << std::endl;
      autopilot_helper_.sendPoseCommand(ring_traj.points.front().position, 0.0);

      // Wait for autopilot to go to got to pose state
      std::cout << "Waiting for Trajectory control state" << std::endl;
      autopilot_helper_.waitForSpecificAutopilotState(
          autopilot::States::TRAJECTORY_CONTROL, 20.0, kExecLoopRate_);

      // Wait for autopilot to go back to hover
      std::cout << "Waiting for hover" << std::endl;
      autopilot_helper_.waitForSpecificAutopilotState(
          autopilot::States::HOVER, 100.0, kExecLoopRate_);

      // Send entire trajectory
      std::cout << "Sending trajectory" << std::endl;
      autopilot_helper_.sendTrajectory(ring_traj);
    }
  exitFlag = false;
}

double SetTrajectory::RandomDouble(double a, double b) {
    double random = ((double) rand()) / (double) RAND_MAX;
    double diff = b - a;
    double r = random * diff;
    return a + r;
}

 // Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Set Trajectory Node");

  ros::NodeHandle n;

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  SetTrajectory *st = new SetTrajectory(n);

  dynamic_reconfigure::Server<rqt_quad_control_tuning::setTrajectoryConfig> srv;
  dynamic_reconfigure::Server<rqt_quad_control_tuning::setTrajectoryConfig>::CallbackType f;
  f = boost::bind(&SetTrajectory::trajectoryCallback, st, _1, _2);
  srv.setCallback(f);

  // Spin
  ros::spin();
  return 0;
}