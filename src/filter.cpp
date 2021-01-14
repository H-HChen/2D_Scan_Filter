#include <filter.h>

using namespace scan_filter;

void scan_filter2d::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr scan){
    if (box_update(*scan, msg_))
    {

        pc_pub->publish(msg_);}

}
bool scan_filter2d::angle_update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan){
        
    filtered_scan.ranges.resize(input_scan.ranges.size());
    filtered_scan.intensities.resize(input_scan.intensities.size());

    double start_angle = input_scan.angle_min;
    double current_angle = input_scan.angle_min;
    builtin_interfaces::msg::Time start_time = input_scan.header.stamp;
    unsigned int count = 0;
    //RCLCPP_INFO(logging_interface_->get_logger(), "processing...");
    //loop through the scan and truncate the beginning and the end of the scan as necessary
    for(unsigned int i = 0; i < input_scan.ranges.size(); ++i){
        //wait until we get to our desired starting angle
        if(start_angle < lower_angle_){
        start_angle += input_scan.angle_increment;
        current_angle += input_scan.angle_increment;
        start_time.set__sec(start_time.sec + input_scan.time_increment);
        }
        else{
        filtered_scan.ranges[count] = input_scan.ranges[i];

        //make sure  that we don't update intensity data if its not available
        if(input_scan.intensities.size() > i)
            filtered_scan.intensities[count] = input_scan.intensities[i];

        count++;

        //check if we need to break out of the loop, basically if the next increment will put us over the threshold
        if(current_angle + input_scan.angle_increment > upper_angle_){
            break;
        }

        current_angle += input_scan.angle_increment;

        }
    }

    //make sure to set all the needed fields on the filtered scan
    filtered_scan.header.frame_id = input_scan.header.frame_id;
    filtered_scan.header.stamp = start_time;
    filtered_scan.angle_min = start_angle;
    filtered_scan.angle_max = current_angle;
    filtered_scan.angle_increment = input_scan.angle_increment;
    filtered_scan.time_increment = input_scan.time_increment;
    filtered_scan.scan_time = input_scan.scan_time;
    filtered_scan.range_min = input_scan.range_min;
    filtered_scan.range_max = input_scan.range_max;

    filtered_scan.ranges.resize(count);

    if(input_scan.intensities.size() >= count)
        filtered_scan.intensities.resize(count);

    RCLCPP_DEBUG(get_logger(), "Filtered out %d points from the laser scan.", (int)input_scan.ranges.size() - (int)count);

    return true;

    }

    bool scan_filter2d::box_update(
        const sensor_msgs::msg::LaserScan &input_scan,
        sensor_msgs::msg::LaserScan &output_scan)
    {
      using namespace std::chrono_literals;
      output_scan = input_scan;
      sensor_msgs::msg::PointCloud2 laser_cloud;

      std::string error_msg;

      bool success = buffer->canTransform(
          box_frame_,
          input_scan.header.frame_id,
          rclcpp::Time(input_scan.header.stamp) + std::chrono::duration<double>(input_scan.ranges.size() * input_scan.time_increment),
          1.0s,
          &error_msg);
      if (!success)
      {
        RCLCPP_WARN(get_logger(), "Could not get transform, irgnoring laser scan! %s", error_msg.c_str());
        return false;
      }

      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      try
      {
        projector_.transformLaserScanToPointCloud(box_frame_, input_scan, laser_cloud, *buffer);
      }
      catch (tf2::TransformException &ex)
      {
        
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, .3, "Ignoring Scan: Waiting for TF");
        
        return false;
      }

      sensor_msgs::PointCloud2ConstIterator<int> iter_i(laser_cloud, "index");
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(laser_cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(laser_cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(laser_cloud, "z");      
      
      if (
        !(iter_i != iter_i.end()) || 
        !(iter_x != iter_x.end()) || 
        !(iter_y != iter_y.end()) || 
        !(iter_z != iter_z.end()))
      {
        RCLCPP_INFO_THROTTLE(get_logger(), steady_clock, .3, "x, y, z and index fields are required, skipping scan");
      }

    for (;
         iter_x != iter_x.end() &&
         iter_y != iter_y.end() &&
         iter_z != iter_z.end() &&
         iter_i != iter_i.end();
         ++iter_x, ++iter_y, ++iter_z, ++iter_i)
    {
        Point point(*iter_x, *iter_y, *iter_z);
        //std::cout<<point.x()<<", "<<point.y()<<", "<<point.z()<<std::endl;
        if (inBox(point))
        {
          output_scan.ranges[*iter_i] = std::numeric_limits<float>::quiet_NaN();
        }
    }

      return true;
    }

    bool scan_filter2d::inBox(Point &point)
    {
      return point.x() < max_.x() && point.x() > min_.x() &&
             point.y() < max_.y() && point.y() > min_.y() &&
             point.z() < max_.z() && point.z() > min_.z();
    }

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto filter_node = std::make_shared<scan_filter::scan_filter2d>();

    rclcpp::spin(filter_node);
    
}