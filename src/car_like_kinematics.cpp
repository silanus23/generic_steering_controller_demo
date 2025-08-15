#include "carlikebot_bringup/car_like_kinematics.hpp"

#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace carlikebot_bringup
{
void CarLikeKinematics::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::vector<std::string> & traction_joint_names,
  const std::vector<std::string> & steering_joint_names)
{
  (void)traction_joint_names;
  (void)steering_joint_names;
}

void CarLikeKinematics::update_reference(const geometry_msgs::msg::Twist & twist_msg, const rclcpp::Time & /*time*/)
{
  reference_twist_ = twist_msg;
}

void CarLikeKinematics::update_measurements(){
   measured_wheel_velocity_left = states["rear_left_wheel_joint/velocity"];
   measured_wheel_velocity_right = states["rear_right_wheel_joint/velocity"];
   measured_steering_angle_left = states["front_left_wheel_joint/position"];   
   measured_steering_angle_right = states["front_right_wheel_joint/position"]; 
}

std::tuple<std::vector<double>, std::vector<double>> CarLikeKinematics::get_commands(
    const double v_target, const double omega_target,const bool open_loop,
    const bool reduce_wheel_speed_until_steering_reached)
{
    update_measurements();
    RCLCPP_INFO(rclcpp::get_logger("CarLikeKinematics_v"), "%f",v_target);
    RCLCPP_INFO(rclcpp::get_logger("CarLikeKinematics_om"), "%f",omega_target);
    double delta_target = 0.0;
    if (std::abs(v_target) > 1e-3)
    {
        delta_target = std::atan((omega_target * wheelbase_) / v_target);
    }
    const double delta_final_cmd = std::clamp(delta_target, -max_steering_angle_, max_steering_angle_);

    double v_final_target = v_target;
    if (reduce_wheel_speed_until_steering_reached)
    {
        const double measured_steering_avg = (measured_steering_angle_left + measured_steering_angle_right) / 2.0;
        const double steering_error = std::abs(delta_final_cmd - measured_steering_avg);
        const double k_speed = std::max(0.0, 1.0 - (steering_error / max_steering_angle_));
        v_final_target *= k_speed;
    }

    const double v_diff = (omega_target * track_width_) / 2.0;
    const double v_left = v_final_target - v_diff;
    const double v_right = v_final_target + v_diff;

    const double phi_dot_left = v_left / wheel_radius_;
    const double phi_dot_right = v_right / wheel_radius_;

    std::vector<double> steering_commands = {delta_final_cmd, delta_final_cmd};
    std::vector<double> traction_commands = {phi_dot_left, phi_dot_right};

/*
      for (const auto& kv : traction_commands)
  {
    RCLCPP_INFO(rclcpp::get_logger("CarLikeKinematics"), "%f" ,kv);
  }*/
    return std::make_tuple(traction_commands , steering_commands);
}

std::unique_ptr<nav_msgs::msg::Odometry> CarLikeKinematics::get_odometry_message(
    const rclcpp::Duration & period)
{
  //RCLCPP_INFO(rclcpp::get_logger("CarLikeKinematics"), "This is an info log message in the odometry.");

    const double v_left_meas = measured_wheel_velocity_left * wheel_radius_;
    const double v_right_meas = measured_wheel_velocity_right * wheel_radius_;

    const double v_actual = (v_left_meas + v_right_meas) / 2.0;
    const double omega_actual = (v_right_meas - v_left_meas) / track_width_;

    const double dt = period.seconds();
    const double delta_x = v_actual * std::cos(odom_heading_) * dt;
    const double delta_y = v_actual * std::sin(odom_heading_) * dt;
    const double delta_heading = omega_actual * dt;

    odom_x_ += delta_x;
    odom_y_ += delta_y;
    odom_heading_ += delta_heading;

    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
    
    odom_msg->header.stamp = rclcpp::Clock().now(); 
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";

    odom_msg->pose.pose.position.x = odom_x_;
    odom_msg->pose.pose.position.y = odom_y_;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, odom_heading_);
    odom_msg->pose.pose.orientation = tf2::toMsg(q);

    odom_msg->twist.twist.linear.x = v_actual;
    odom_msg->twist.twist.angular.z = omega_actual;

    return odom_msg;
}

}  // namespace carlike_kinematics_plugin

PLUGINLIB_EXPORT_CLASS(
  carlikebot_bringup::CarLikeKinematics, kinematic_model::KinematicModelBase)
