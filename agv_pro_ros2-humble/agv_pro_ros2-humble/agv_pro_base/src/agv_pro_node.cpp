#include "agv_pro_base/agv_pro_driver.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AGV_PRO>("agv_pro_base_node");

  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}