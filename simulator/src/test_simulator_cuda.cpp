#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>

#include <nav_msgs/msg/odometry.hpp>
// #include <pcl_ros/point_cloud.h>
// #include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "maps.hpp"
#include "sensor_simulator.cuh"

using namespace raycast;

class SensorSimulator : public rclcpp::Node {
public:
  SensorSimulator(const rclcpp::NodeOptions &options) : Node("sensor_simulator_node", options) {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("simulator");  // 获取包的share目录
    std::string CONFIG_FILE_PATH = package_share_directory + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(CONFIG_FILE_PATH);
    // 读取camera参数
    camera = new CameraParams();
    camera->fx = config["camera"]["fx"].as<float>();
    camera->fy = config["camera"]["fy"].as<float>();
    camera->cx = config["camera"]["cx"].as<float>();
    camera->cy = config["camera"]["cy"].as<float>();
    camera->image_width = config["camera"]["image_width"].as<int>();
    camera->image_height = config["camera"]["image_height"].as<int>();
    camera->max_depth_dist = config["camera"]["max_depth_dist"].as<float>();
    camera->normalize_depth = config["camera"]["normalize_depth"].as<bool>();
    float pitch = config["camera"]["pitch"].as<float>() * M_PI / 180.0;
    quat_bc = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());

    // 读取lidar参数
    lidar = new LidarParams();
    lidar->vertical_lines = config["lidar"]["vertical_lines"].as<int>();
    lidar->vertical_angle_start = config["lidar"]["vertical_angle_start"].as<float>();
    lidar->vertical_angle_end = config["lidar"]["vertical_angle_end"].as<float>();
    lidar->horizontal_num = config["lidar"]["horizontal_num"].as<int>();
    lidar->horizontal_resolution = config["lidar"]["horizontal_resolution"].as<float>();
    lidar->max_lidar_dist = config["lidar"]["max_lidar_dist"].as<float>();

    render_lidar = config["render_lidar"].as<bool>();
    render_depth = config["render_depth"].as<bool>();
    float depth_fps = config["depth_fps"].as<float>();
    float lidar_fps = config["lidar_fps"].as<float>();

    std::string ply_file = package_share_directory + "/" + config["ply_file"].as<std::string>();
    RCLCPP_INFO(this->get_logger(),
                "Ply file path: %s", ply_file.c_str());
    std::string odom_topic = config["odom_topic"].as<std::string>();
    std::string depth_topic = config["depth_topic"].as<std::string>();
    std::string lidar_topic = config["lidar_topic"].as<std::string>();

    // 读取地图参数
    bool use_random_map = config["random_map"].as<bool>();
    float resolution = config["resolution"].as<float>();
    int occupy_threshold = config["occupy_threshold"].as<int>();
    pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mock_map", 1);
    int seed = config["seed"].as<int>();
    int sizeX = config["x_length"].as<int>();
    int sizeY = config["y_length"].as<int>();
    int sizeZ = config["z_length"].as<int>();
    int type = config["maze_type"].as<int>();
    double scale = 1 / resolution;
    sizeX = sizeX * scale;
    sizeY = sizeY * scale;
    sizeZ = sizeZ * scale;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (use_random_map) {
      printf("1.Generate Random Map... \n");
      mocka::Maps::BasicInfo info;
      info.sizeX = sizeX;
      info.sizeY = sizeY;
      info.sizeZ = sizeZ;
      info.seed = seed;
      info.scale = scale;
      info.cloud = cloud;

      mocka::Maps map;
      map.setParam(config);
      map.setInfo(info);
      map.generate(type);
    } else {
      printf("1.Reading Point Cloud %s... \n", ply_file.c_str());
      if (pcl::io::loadPLYFile(ply_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read PLY file \n");
      }
    }
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "world";

    std::cout << "Pointloud size:" << cloud->points.size() << std::endl;
    printf("2.Mapping... \n");
    grid_map = new GridMap(cloud, resolution, occupy_threshold);

    // ROS
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, 1);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 1, std::bind(&SensorSimulator::odomCallback, this, std::placeholders::_1));
    timer_depth_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / depth_fps),
                                           std::bind(&SensorSimulator::timerDepthCallback, this));
    timer_lidar_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / lidar_fps),
                                           std::bind(&SensorSimulator::timerLidarCallback, this));

    printf("3.Simulation Ready! \n");
    rclcpp::spin(this->get_node_base_interface());
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

  void timerDepthCallback();

  void timerLidarCallback();

  void timerMapCallback();

private:
  bool render_depth{false};
  bool render_lidar{false};
  bool odom_init{false};
  Eigen::Quaternionf quat;
  Eigen::Quaternionf quat_bc, quat_wc;
  Eigen::Vector3f pos;

  CameraParams *camera;
  LidarParams *lidar;
  GridMap *grid_map;
  sensor_msgs::msg::PointCloud2 output;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_depth_;
  rclcpp::TimerBase::SharedPtr timer_lidar_;
  rclcpp::TimerBase::SharedPtr timer_map_;

  // mocka::Maps map;
};

void SensorSimulator::timerDepthCallback() {
  if (!odom_init || !render_depth) return;

  auto start = std::chrono::high_resolution_clock::now();

  cudaMat::SE3<float> T_wc(
    quat_wc.w(), quat_wc.x(), quat_wc.y(), quat_wc.z(), pos.x(), pos.y(), pos.z());
  cv::Mat depth_image;
  renderDepthImage(grid_map, camera, T_wc, depth_image);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  // std::cout << "生成图像耗时: " << elapsed.count() << " 秒" << std::endl;

  sensor_msgs::msg::Image ros_image;
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = this->now();
  cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  cv_image.image = depth_image;
  cv_image.toImageMsg(ros_image);
  image_pub_->publish(ros_image);
}

void SensorSimulator::timerMapCallback() {
  if (pcl_pub_->get_subscription_count() > 0) pcl_pub_->publish(output);
}

void SensorSimulator::timerLidarCallback() {
  if (!odom_init || !render_lidar) return;

  auto start = std::chrono::high_resolution_clock::now();

  cudaMat::SE3<float> T_wc(quat.w(), quat.x(), quat.y(), quat.z(), pos.x(), pos.y(), pos.z());
  pcl::PointCloud<pcl::PointXYZ> lidar_points;
  renderLidarPointcloud(grid_map, lidar, T_wc, lidar_points);

  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  // std::cout << "生成雷达耗时: " << elapsed.count() << " 秒" << std::endl;

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(lidar_points, output);
  output.header.stamp = this->now();
  output.header.frame_id = "odom";
  point_cloud_pub_->publish(output);
}

void SensorSimulator::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  quat.x() = msg->pose.pose.orientation.x;
  quat.y() = msg->pose.pose.orientation.y;
  quat.z() = msg->pose.pose.orientation.z;
  quat.w() = msg->pose.pose.orientation.w;
  quat_wc = quat * quat_bc;

  pos.x() = msg->pose.pose.position.x;
  pos.y() = msg->pose.pose.position.y;
  pos.z() = msg->pose.pose.position.z;

  odom_init = true;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SensorSimulator)

// int main(int argc, char **argv) {
//   ros::init(argc, argv, "sensor_simulator_node");
//   ros::NodeHandle nh;

//   SensorSimulator sensor_simulator(nh);
//   return 0;
// }
