#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node {
public:
	MyNode() : Node("my_node1") {
		RCLCPP_INFO(this->get_logger(), "Hello from my_node!");
	}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<MyNode>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
