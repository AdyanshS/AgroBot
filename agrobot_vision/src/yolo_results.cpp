#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class YoloResults : public rclcpp::Node
{
  public:
    YoloResults()
    : Node("yolo_results")
    {
      this->declare_parameter<std::string>("example_param", "default_value");
      std::string example_param = this->get_parameter("example_param").as_string();
      RCLCPP_INFO(this->get_logger(), "Declared parameter 'example_param'. Value: %s", example_param.c_str());

      RCLCPP_INFO(this->get_logger(), "Hello world from the C++ node %s", "yolo_results");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<YoloResults>());
  rclcpp::shutdown();
  return 0;
}