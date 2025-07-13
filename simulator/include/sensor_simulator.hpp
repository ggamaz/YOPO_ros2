#ifndef SENSOR_SIMULATOR_H
#define SENSOR_SIMULATOR_H
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// #include <pcl_ros/point_cloud.h>
#include <cv_bridge/cv_bridge.h>
#include <omp.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>  // 包含这个头文件
#include <chrono>
#include <rclcpp/rclcpp.hpp>

class SensorSimulator : public rclcpp::Node {
public:
  SensorSimulator(const rclcpp::NodeOptions &options) : Node("sensor_simulator_node", options) {
    std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("simulator");  // 获取包的share目录
    std::string CONFIG_FILE_PATH = package_share_directory + "/config/config.yaml";
    YAML::Node config = YAML::LoadFile(CONFIG_FILE_PATH);
    // 读取camera参数
    fx = config["camera"]["fx"].as<float>();
    fy = config["camera"]["fy"].as<float>();
    cx = config["camera"]["cx"].as<float>();
    cy = config["camera"]["cy"].as<float>();
    image_width = config["camera"]["image_width"].as<int>();
    image_height = config["camera"]["image_height"].as<int>();
    max_depth_dist = config["camera"]["max_depth_dist"].as<float>();
    normalize_depth = config["camera"]["normalize_depth"].as<bool>();

    // 读取lidar参数
    vertical_lines = config["lidar"]["vertical_lines"].as<int>();
    vertical_angle_start = config["lidar"]["vertical_angle_start"].as<float>();
    vertical_angle_end = config["lidar"]["vertical_angle_end"].as<float>();
    horizontal_num = config["lidar"]["horizontal_num"].as<int>();
    horizontal_resolution = config["lidar"]["horizontal_resolution"].as<float>();
    max_lidar_dist = config["lidar"]["max_lidar_dist"].as<float>();

    render_lidar = config["render_lidar"].as<bool>();
    render_depth = config["render_depth"].as<bool>();
    float depth_fps = config["depth_fps"].as<float>();
    float lidar_fps = config["lidar_fps"].as<float>();

    std::string ply_file = package_share_directory + "/" + config["ply_file"].as<std::string>();
    std::string odom_topic = config["odom_topic"].as<std::string>();
    std::string depth_topic = config["depth_topic"].as<std::string>();
    std::string lidar_topic = config["lidar_topic"].as<std::string>();

    float resolution = config["resolution"].as<float>();
    int expand_y_times = config["expand_y_times"].as<int>();
    int expand_x_times = config["expand_x_times"].as<int>();

    pcl::PointCloud<pcl::PointXYZ>::Ptr orig_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    RCLCPP_INFO(this->get_logger(), "1.Reading Point Cloud... \n");

    // if (pcl::io::loadPCDFile("/home/lu/用完删除/test/map.pcd", *cloud) == -1) {
    //     PCL_ERROR("Couldn't read PLY file \n");
    //     return;
    // }

    if (pcl::io::loadPLYFile(ply_file, *orig_cloud) == -1) {
      PCL_ERROR("Couldn't read PLY file \n");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "2.Processing... \n");

    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    *cloud = *orig_cloud;
    for (int i = 0; i < expand_x_times; ++i) {
      expand_cloud(cloud, 0);
    }
    for (int i = 0; i < expand_y_times; ++i) {
      expand_cloud(cloud, 1);
    }

    octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr(
      new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(resolution));
    octree->setInputCloud(cloud);
    octree->addPointsFromInputCloud();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic, 1);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 1);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 1, std::bind(&SensorSimulator::odomCallback, this, std::placeholders::_1));

    timer_depth_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / depth_fps),
                                           std::bind(&SensorSimulator::timerDepthCallback, this));
    timer_lidar_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / lidar_fps),
                                           std::bind(&SensorSimulator::timerLidarCallback, this));
    RCLCPP_INFO(this->get_logger(), "3.Simulation Ready! \n");
    rclcpp::spin(this->get_node_base_interface());
  }

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);

  cv::Mat renderDepthImage();

  pcl::PointCloud<pcl::PointXYZ> renderLidarPointcloud();

  void timerDepthCallback();

  void timerLidarCallback();

  void expand_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud, int direction = 0);

private:
  bool render_depth{false};
  bool render_lidar{false};
  bool odom_init{false};
  Eigen::Quaternionf quat;
  Eigen::Vector3f pos;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr octree;

  // camera param
  float fx = 80.0f;  // focal length x
  float fy = 80.0f;  // focal length y
  float cx = 80.0f;  // principal point x (image center)
  float cy = 45.0f;  // principal point y (image center)
  int image_width = 160;
  int image_height = 90;
  float max_depth_dist{20};
  bool normalize_depth{false};

  // lidar param
  int vertical_lines = 16;             // 纵向16线
  float vertical_angle_start = -15.0;  // 起始垂直角度
  float vertical_angle_end = 15.0;     // 结束垂直角度
  int horizontal_num = 360;            // 水平360点
  float horizontal_resolution = 1.0;   // 水平分辨率为1度
  float max_lidar_dist{50};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_depth_;
  rclcpp::TimerBase::SharedPtr timer_lidar_;
};

#endif  //SENSOR_SIMULATOR_H
