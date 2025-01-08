#include <rclcpp/rclcpp.hpp>
#include "../include/dxl_controller/serial_process.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialProcess>());
  rclcpp::shutdown();
  return 0;
}