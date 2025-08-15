#ifndef CARLIKEBOT_BRINGUP__CAR_LIKE_KINEMATICS_HPP_
#define CARLIKEBOT_BRINGUP__CAR_LIKE_KINEMATICS_HPP_

#include "generic_steering_controller/kinematic_model_base.hpp"
//#include "carlikebot_bringup/carlikebot_bringup_parameters.hpp"

#include <memory>
#include <string>
#include <vector>

namespace carlikebot_bringup
{
class CarLikeKinematics : public kinematic_model::KinematicModelBase
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::vector<std::string> & traction_joint_names,
    const std::vector<std::string> & steering_joint_names) override;

  void update_reference(
    const geometry_msgs::msg::Twist & twist_msg, const rclcpp::Time & time) override;


  std::unique_ptr<nav_msgs::msg::Odometry> get_odometry_message(
    const rclcpp::Duration & period) override;

  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    const double v_target, const double omega_target, const bool open_loop,
    const bool reduce_wheel_speed_until_steering_reached) override;

  void update_measurements();

private:

  double odom_x_ = 0.0;
  double odom_y_ = 0.0;
  double odom_heading_ = 0.0;
  double measured_wheel_velocity_left = 0.0;
  double measured_wheel_velocity_right = 0.0;
  double measured_steering_angle_right = 0.0;
  double measured_steering_angle_left = 0.0;
  geometry_msgs::msg::Twist reference_twist_;
  const double wheelbase_ = 0.325;
  const double track_width_ = 0.260; // From base_width in URDF
  const double wheel_radius_ = 0.05;
  const double max_steering_angle_ = 0.4; // From joint limits in URDF

//  std::shared_ptr<generic_steering_controller_parameters::ParamListener> param_listener_;
//  generic_steering_controller_parameters::Params params_;

};

}  // namespace carlikebot_bringup
#endif
