#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <touch_info/msg/touch.hpp>
namespace touch{
  class publisher : public rclcpp::Node {
    private:
      rclcpp::Publisher<touch_info::msg::Touch>::SharedPtr touch_pub;
      
    public:
      publisher(const std::string& name);
      void publish(double x, double y, double z, double roll, double pitch, double yaw, bool button1, bool button2);
  
  
  };

}
