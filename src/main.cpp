#include "rclcpp/rclcpp.hpp"
#include "dstereo_occnet/dstereo_occnet_node.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DStereoOccNetNode>("dstereo_occnet_node", rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}