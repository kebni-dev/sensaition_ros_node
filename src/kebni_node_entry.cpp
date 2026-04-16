
#include "kebni_driver/kebni_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<KebniNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
