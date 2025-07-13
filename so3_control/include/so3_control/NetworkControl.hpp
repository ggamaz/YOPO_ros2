#ifndef NETWORK_CONTROL_H_
#define NETWORK_CONTROL_H_

#include <Eigen/Eigen>
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <quadrotor_msgs/srv/set_takeoff_land.hpp>

#include <sensor_msgs/msg/imu.hpp>
// #include <tf/transform_datatypes.h>
#include <tf2/transform_datatypes.hpp>

// #include <ros/ros.h>
// #include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
// #include <regex>
#include <so3_control/HGDO.hpp>
#include <so3_control/SO3Control.hpp>
#include <so3_control/mavros_interface.hpp>
#include <string>
#include <thread>

#include <chrono>
using namespace std::chrono_literals;
#define ONE_G 9.81

class NetworkControl : public rclcpp::Node {
public:
  NetworkControl(const rclcpp::NodeOptions &options)
      : Node("network_ctrl_node", options) {

    so3_controller_.setMass(mass_);
    disturbance_observer_ = HGDO(control_dt_);

    is_simulation_ = this->declare_parameter("is_simulation", false);
    use_disturbance_observer_ =
        this->declare_parameter("use_disturbance_observer", false);
    hover_thrust_ = this->declare_parameter("hover_thrust", 0.4);
    kx_xy = this->declare_parameter("kx_xy", 5.7);
    kx_z = this->declare_parameter("kx_z", 6.2);
    kv_xy = this->declare_parameter("kv_xy", 3.4);
    kv_z = this->declare_parameter("kv_z", 4.0);
    record_log_ = this->declare_parameter("record_log", false);
    logger_file_name =
        this->declare_parameter("logger_file_name", std::string("/home/lu/"));
    printf("kx: (%f, %f, %f), kv: (%f, %f, %f) \n", kx_xy, kx_xy, kx_z, kv_xy,
           kv_xy, kv_z);

    so3_command_pub_ =
        this->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

    position_cmd_sub_ =
        this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "position_cmd", 1,
            std::bind(&NetworkControl::network_cmd_callback, this,
                      std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1,
        std::bind(&NetworkControl::odom_callback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 1,
        std::bind(&NetworkControl::imu_callback, this, std::placeholders::_1));

    takeoff_land_control_timer =
        this->create_wall_timer(std::chrono::duration<double>(control_dt_),
                               std::bind(&NetworkControl::timerCallback,
                               this));

    takeoff_land_srv =
        this->create_service<quadrotor_msgs::srv::SetTakeoffLand>(
            "takeoff_land",
            std::bind(&NetworkControl::takeoff_land_srv_handle, this,
                      std::placeholders::_1, std::placeholders::_2));

    if (is_simulation_) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      std::thread(&NetworkControl::simulateTakeoff, this).detach();
    }
  };

  ~NetworkControl(){};

private:
  rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr
      so3_command_pub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr
      position_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<quadrotor_msgs::srv::SetTakeoffLand>::SharedPtr
      takeoff_land_srv;
  rclcpp::TimerBase::SharedPtr takeoff_land_control_timer;
  std::mutex mutex_;

  double mass_ = 0.98;
  double control_dt_ = 0.02;
  double hover_thrust_ = 0.4;
  double kx_xy, kx_z, kv_xy, kv_z;

  double cur_yaw_ = 0;
  Eigen::Vector3d cur_pos_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d cur_vel_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d cur_acc_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Quaterniond cur_att_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d dis_acc_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d last_des_acc_ = Eigen::Vector3d(0, 0, 0);
  double last_thrust_ = 0;

  Eigen::Vector3d des_pos_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d des_vel_ = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d des_acc_ = Eigen::Vector3d(0, 0, 0);
  double des_yaw_ = 0;
  double des_yaw_dot_ = 0;

  bool is_simulation_ = false;
  bool state_init_ = false;
  bool ref_valid_ = false;
  bool ctrl_valid_ = false;
  bool position_cmd_init_ = false;
  bool takeoff_cmd_init_ = false;
  bool use_disturbance_observer_ = false;
  bool record_log_ = false;

  SO3Control so3_controller_;
  HGDO disturbance_observer_;
  Mavros_Interface mavros_interface_;

  std::ofstream logger;
  std::string logger_file_name;

  void initLogRecorder();

  void recordLog(Eigen::Vector3d &cur_v, Eigen::Vector3d &cur_a,
                 Eigen::Vector3d &des_a, Eigen::Vector3d &dis_a, double cur_yaw,
                 double des_yaw);

  Eigen::Vector3d publishHoverSO3Command(Eigen::Vector3d des_pos_,
                                         Eigen::Vector3d des_vel_,
                                         Eigen::Vector3d des_acc_,
                                         double des_yaw_, double des_yaw_dot_);

  void get_Q_from_ACC(const Eigen::Vector3d &ref_acc, double ref_yaw,
                      Eigen::Quaterniond &quat_des, Eigen::Vector3d &force_des);

  void pub_SO3_command(Eigen::Vector3d ref_acc, double ref_yaw, double cur_yaw);

  void limite_acc(Eigen::Vector3d &acc);

  void network_cmd_callback(
      const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr &cmd);

  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr &odom);

  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr &imu);
  void timerCallback();

  // mavros interface
  void takeoff_land_srv_handle(
      const quadrotor_msgs::srv::SetTakeoffLand::Request::SharedPtr &req,
      quadrotor_msgs::srv::SetTakeoffLand::Response::SharedPtr res) {
    std::thread t(&NetworkControl::takeoff_land_thread, this, req);
    t.detach();
    res->res = true;
  }

  bool arm_disarm_vehicle(bool arm);

  void takeoff_land_thread(
      const quadrotor_msgs::srv::SetTakeoffLand::Request::SharedPtr &req);

  void simulateTakeoff() {
    rclcpp::Client<quadrotor_msgs::srv::SetTakeoffLand>::SharedPtr client =
        this->create_client<quadrotor_msgs::srv::SetTakeoffLand>(
            "takeoff_land");
    quadrotor_msgs::srv::SetTakeoffLand::Request::SharedPtr srv =
        std::make_shared<quadrotor_msgs::srv::SetTakeoffLand::Request>();
    srv->takeoff = true;
    srv->takeoff_altitude = 2.0;

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto result = client->async_send_request(srv);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("NetworkControl"),
                  "Takeoff called successfully");
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("NetworkControl"),
                   "Failed to call takeoff service");
    }
  }
};

#endif