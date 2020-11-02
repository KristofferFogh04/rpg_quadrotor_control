#include <rqt_quad_control_tuning/set_trajectory.h>



SetTrajectory::SetTrajectory(ros::NodeHandle& nh_) : // default constructor for
nh_(nh_)
{
  std::cout << "Not implemented" << std::endl;
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
    std::cout << "Waypoints not implemented" << std::endl;
    break;
  case 2:
    std::cout << "Horizontal Circle" << std::endl;
    SetTrajectory::computeCircularTrajectory(config.speed, config.circle_radius, config.num_rotations);
    break;
  case 3:
    std::cout << "Vertical Circle" << std::endl;
    SetTrajectory::computeVerticalCircularTrajectory(config.speed, config.circle_radius, config.num_rotations, config.circle_orientation);
    break;
  case 4:
    std::cout << "Not implemented" << std::endl;
    break;
  case 5:
    std::cout << "Not implemented" << std::endl;
    break;
  case 6:
    std::cout << "Not implemented" << std::endl;
    break;
  case 7:
    std::cout << "Not implemented" << std::endl;
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
  const Eigen::Vector3d center = Eigen::Vector3d(-2.0, 0.0, 10);

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::circles::computeHorizontalCircleTrajectory(
          center, radius, max_vel, 0.0, num_rotations*2*M_PI, kExecLoopRate_);

  // First send go to command to get the drone to the start point
  // Send pose command
  autopilot_helper_.sendPoseCommand(manual_traj.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  while (ros::ok() && !manual_traj.points.empty()) {
    autopilot_helper_.sendReferenceState(manual_traj.points.front());
    manual_traj.points.pop_front();
    ros::spinOnce();
    command_rate.sleep();
    }
}

void SetTrajectory::computeVerticalCircularTrajectory(
    double max_vel, 
    double radius, 
    int num_rotations, 
    double orientation){

  ros::Rate command_rate(kExecLoopRate_);

  // Generate trajectory, sample it and send it as reference states
  const Eigen::Vector3d center = Eigen::Vector3d(-2.0, 0.0, 10);

  quadrotor_common::Trajectory manual_traj =
      trajectory_generation_helper::circles::computeVerticalCircleTrajectory(
          center, orientation, radius, max_vel, 0.0, num_rotations*2*M_PI, kExecLoopRate_);

  // First send go to command to get the drone to the start point
  // Send pose command
  autopilot_helper_.sendPoseCommand(manual_traj.points.front().position, 0.0);

  // Wait for autopilot to go to got to pose state
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::TRAJECTORY_CONTROL, 2.0, kExecLoopRate_);

  // Wait for autopilot to go back to hover
  autopilot_helper_.waitForSpecificAutopilotState(
      autopilot::States::HOVER, 10.0, kExecLoopRate_);

  while (ros::ok() && !manual_traj.points.empty()) {
    autopilot_helper_.sendReferenceState(manual_traj.points.front());
    manual_traj.points.pop_front();
    ros::spinOnce();
    command_rate.sleep();
    }
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