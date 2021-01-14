#include <rclcpp/rclcpp.hpp>
#include <laser_filters/angular_bounds_filter.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <iostream>
#include <laser_geometry/laser_geometry.hpp>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
 
using namespace laser_filters;
typedef tf2::Vector3 Point;
namespace scan_filter
{
  class scan_filter2d : public rclcpp::Node
    {
    public:
      scan_filter2d(): Node("scan_filter2d")
      {
        pc_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/pointcloud", 5);
        laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan",rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
          std::bind(&scan_filter2d::LaserCallBack, this, std::placeholders::_1));
        buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*buffer);
        max_.setValue(5.0, 5.0 ,5.0);
        min_.setValue(0.0, 0.0, -1.0);
      }
      sensor_msgs::msg::LaserScan msg_;


    private:
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr  laser_sub;
      rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr  pc_pub;
      void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan);
      bool angle_update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan);
      bool box_update(const sensor_msgs::msg::LaserScan &input_scan,sensor_msgs::msg::LaserScan &output_scan);
      bool inBox(Point &point);
      std::shared_ptr<tf2_ros::Buffer> buffer;
      std::shared_ptr<tf2_ros::TransformListener> tf_listener;
      laser_geometry::LaserProjection projector_;
      double lower_angle_ = 0.0;
      double upper_angle_ = 150.0;
      std::string box_frame_ = "laser_frame";
      Point max_;
      Point min_;


 
  };
}