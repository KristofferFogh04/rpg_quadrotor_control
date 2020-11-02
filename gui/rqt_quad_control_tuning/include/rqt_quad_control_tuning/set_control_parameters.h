#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <extra_msgs/ParameterCommand.h>
#include <dynamic_reconfigure/server.h>
#include <rqt_quad_control_tuning/setControlParametersConfig.h>
#include <rqt_quad_control_tuning/setTrajectoryConfig.h>
#include <autopilot/autopilot_helper.h>
#include <trajectory_generation_helper/heading_trajectory_helper.h>
#include <trajectory_generation_helper/circle_trajectory_helper.h>
#include <trajectory_generation_helper/polynomial_trajectory_helper.h>

class SetControlParameters
{

 public:

  SetControlParameters(ros::NodeHandle& nh_);

  void controlParametersCallback(
    rqt_quad_control_tuning::setControlParametersConfig &config,
    uint32_t level);

  void publishParameters(extra_msgs::ParameterCommand);
  
 private:

  ros::NodeHandle nh_; 
  ros::Publisher parameterPublisher_;

  extra_msgs::ParameterCommand target_cmd_;

};  // End of Class
