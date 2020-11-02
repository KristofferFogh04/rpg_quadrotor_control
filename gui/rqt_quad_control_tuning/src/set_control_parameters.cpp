#include <rqt_quad_control_tuning/set_control_parameters.h>



SetControlParameters::SetControlParameters(ros::NodeHandle& nh_) : // default constructor for
nh_(nh_)
{

  // ROS publishers:
  parameterPublisher_ = nh_.advertise<extra_msgs::ParameterCommand>("/setControlParameters", 10);
}


void SetControlParameters::publishParameters(extra_msgs::ParameterCommand msg){

  parameterPublisher_.publish(msg);
}

void SetControlParameters::controlParametersCallback(
  rqt_quad_control_tuning::setControlParametersConfig &config,
  uint32_t  level) {

  target_cmd_.kpxy = config.kpxy;
  target_cmd_.kdxy = config.kdxy;
  target_cmd_.kpz = config.kpz;
  target_cmd_.kdz = config.kdz;
  target_cmd_.krp = config.krp;
  target_cmd_.kyaw = config.kyaw;
  target_cmd_.perform_aerodynamics_compensation = config.rotor_drag;
  target_cmd_.k_drag_x = config.k_drag_xw;
  target_cmd_.k_drag_y = config.k_drag_y;
  target_cmd_.k_drag_z = config.k_drag_z;
  target_cmd_.k_thrust_horz = config.k_thrust_horz;

  SetControlParameters::publishParameters(target_cmd_);

}

 // Main node
int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "Set Control Parameters Node");

  ros::NodeHandle n;

  // Implement with a defined waypoints: start waypoint, land waypoint and a list of gate waypoints
  SetControlParameters *scp = new SetControlParameters(n);

  dynamic_reconfigure::Server<rqt_quad_control_tuning::setControlParametersConfig> srv;
  dynamic_reconfigure::Server<rqt_quad_control_tuning::setControlParametersConfig>::CallbackType f;
  f = boost::bind(&SetControlParameters::controlParametersCallback, scp, _1, _2);
  srv.setCallback(f);

  // Spin
  ros::spin();
  return 0;
}
