#ifndef MAVROS_INTERFACE_H_
#define MAVROS_INTERFACE_H_

// #include "ros/ros.h"
#include <stdlib.h>
#include <string>

#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

// #include <eigen3/Eigen/Core>
// #include <eigen3/Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Geometry>

class Mavros_Interface {
public:
  Mavros_Interface() {
    rclcpp::NodeOptions options;
    this->_nh = rclcpp::Node::make_shared("mavros_interface", options);
    _state.reset();
    std::string base_name = "/mavros";
    // char id_str[10];
    // sprintf(id_str, "%d", id);
    // base_name += id_str;

    std::string att_target_pub_name;
    att_target_pub_name = base_name + "/setpoint_raw/attitude";

    att_target_pub = _nh->create_publisher<mavros_msgs::msg::AttitudeTarget>(
        att_target_pub_name, 10);

    std::string state_sub_name;
    state_sub_name = base_name + "/state";
    state_sub = _nh->create_subscription<mavros_msgs::msg::State>(
        state_sub_name, 10,
        std::bind(&Mavros_Interface::state_cb, this, std::placeholders::_1));

    std::string set_mode_s_name;
    set_mode_s_name = base_name + "/set_mode";
    set_mode_client =
        _nh->create_client<mavros_msgs::srv::SetMode>(set_mode_s_name);

    std::string arm_disarm_s_name;
    arm_disarm_s_name = base_name + "/cmd/arming";
    arm_disarm_client =
        _nh->create_client<mavros_msgs::srv::CommandBool>(arm_disarm_s_name);
  }

  ~Mavros_Interface() {}

  struct mavros_state_t {
    rclcpp::Time header;
    bool has_armed;
    bool offboard_enabled;
    void reset() {
      has_armed = false;
      offboard_enabled = false;
    }
    mavros_state_t() { reset(); }
  };

  void state_cb(const mavros_msgs::msg::State &state_data) {
    mavros_msgs::msg::State temp_data = state_data;
    _state.header = state_data.header.stamp;
    _state.has_armed = state_data.armed;
    if (state_data.mode == "OFFBOARD") {
      _state.offboard_enabled = true;
    } else {
      _state.offboard_enabled = false;
    }
  }

  void get_status(bool &arm_state, bool &offboard_enabled) {
    arm_state = _state.has_armed;
    offboard_enabled = _state.offboard_enabled;
  }

  bool set_arm_and_offboard() {
    rclcpp::Rate _ofb_check_rate(1);
    int try_arm_ofb_times = 0;
    while (!_state.offboard_enabled || !_state.has_armed) {
      if (_state.offboard_enabled) {
        rclcpp::Rate _arm_check_rate(1);
        while (!_state.has_armed) {
          mavros_msgs::srv::CommandBool::Request::SharedPtr arm_srv =
              std::make_shared<mavros_msgs::srv::CommandBool::Request>();
          arm_srv->value = true;

          auto result = arm_disarm_client->async_send_request(arm_srv);
          if (rclcpp::spin_until_future_complete(_nh, result) ==
              rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(rclcpp::get_logger("Mavros_Interface"),
                        "vehicle ARMED");
          }
          try_arm_ofb_times = try_arm_ofb_times + 1;
          if (try_arm_ofb_times >= 3) {
            RCLCPP_ERROR(rclcpp::get_logger("Mavros_Interface"),
                         "try 3 times, cannot armed uav, give up!");
            return false;
          }
          _arm_check_rate.sleep();
        }
      } else {
        RCLCPP_INFO(rclcpp::get_logger("Mavros_Interface"),
                    "not in OFFBOARD mode");
        mavros_msgs::srv::SetMode::Request::SharedPtr set_mode_srv =
            std::make_shared<mavros_msgs::srv::SetMode::Request>();
        set_mode_srv->base_mode = 0;
        set_mode_srv->custom_mode = "OFFBOARD";

        auto result = set_mode_client->async_send_request(set_mode_srv);
        if (rclcpp::spin_until_future_complete(_nh, result) !=
            rclcpp::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(rclcpp::get_logger("Mavros_Interface"),
                       "Failed to set mode to OFFBOARD");
          return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("Mavros_Interface"),
                    "switch to OFFBOARD mode");
        _ofb_check_rate.sleep();
      }
    }
    return true;
  }

  /* void set_arm(const ros::TimerEvent& event) {
      if(_state.offboard_enabled) {
          mavros_msgs::CommandBool arm_srv;
          arm_srv.request.value = true;
          if (arm_disarm_client.call(arm_srv)) {
              ROS_INFO("vehicle ARMED");
          }
      } else {
          ROS_INFO("not in OFFBOARD mode");
      }
  } */

  bool set_disarm() {
    rclcpp::Rate _arm_check_rate(1);
    while (_state.has_armed) {
      mavros_msgs::srv::CommandBool::Request::SharedPtr arm_srv =
          std::make_shared<mavros_msgs::srv::CommandBool::Request>();
      arm_srv->value = false;

      auto result = arm_disarm_client->async_send_request(arm_srv);
      if (rclcpp::spin_until_future_complete(_nh, result) !=
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(rclcpp::get_logger("Mavros_Interface"),
                     "Failed to disarm vehicle");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("Mavros_Interface"), "vehicle DISARMED");
      _arm_check_rate.sleep();
    }
    return true;
  }

  void pub_att_thrust_cmd(const Eigen::Quaterniond &q_d,
                          const double &thrust_d) {
    /*
        目前mavros用的是北东地的坐标系，为了和pid通用所以没有改mavros而是在这里改为北东地
    */
    mavros_msgs::msg::AttitudeTarget at_cmd;
    at_cmd.header.stamp = rclcpp::Clock().now();
    at_cmd.type_mask = at_cmd.IGNORE_ROLL_RATE | at_cmd.IGNORE_PITCH_RATE |
                       at_cmd.IGNORE_YAW_RATE;
    at_cmd.thrust = (float)thrust_d;
    at_cmd.orientation.w = q_d.w();
    at_cmd.orientation.x = q_d.x();
    at_cmd.orientation.y = -q_d.y();
    at_cmd.orientation.z = -q_d.z();
    att_target_pub->publish(at_cmd);
  }

  // for simulation
  void set_arm_and_offboard_manually() {
    _state.has_armed = true;
    _state.offboard_enabled = true;
  }

  void set_disarm_manually() { _state.has_armed = false; }

private:
  rclcpp::Node::SharedPtr _nh;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr att_target_pub;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_disarm_client;

  mavros_state_t _state;
};

#endif
