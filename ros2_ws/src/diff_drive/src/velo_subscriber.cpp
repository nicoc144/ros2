#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class VeloSubscriber : public rclcpp::Node {
	public:
		VeloSubscriber() : Node("velocity_sub") {
			subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("topic", 1,
			std::bind(&VeloSubscriber::topic_callback, this, _1));
		}
		
	private:
		void topic_callback(const geometry_msgs::msg::Twist::SharedPtr message) const {
			RCLCPP_INFO(this->get_logger(), "Recieved X:%lf, Y:%lf, Z:%lf",
				// "message" is a pointer to the data, use arrow operator to dereference
				message->linear.x,
				message->linear.y,
				message->linear.z
			);
		}
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv); // Initialize ROS2 communications
	rclcpp::spin(std::make_shared<VeloSubscriber>()); // 'spin()' means run the node indefinitely until it's killed
	rclcpp::shutdown(); // End ROS2 communications
	return 0;
}