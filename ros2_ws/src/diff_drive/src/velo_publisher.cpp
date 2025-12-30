#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VeloPublisher : public rclcpp::Node {
	public:
		VeloPublisher() : Node("velocity_pub") {
			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("topic", 1);
			timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
				std::bind(&VeloPublisher::velo_callback, this)
			);
		}

	private:
		void velo_callback() {
			// Twist ia a message type for expressing velocity linear/angular
			auto message = geometry_msgs::msg::Twist(); 
			message.linear.x = 1.1;
			message.linear.y = 1.2;
			message.linear.z = 1.3;
			RCLCPP_INFO(this->get_logger(), "Publishing: %lf, %lf, %lf",
				message.linear.x, message.linear.y, message.linear.z);
			publisher_->publish(message);
		}
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
		rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv); // Initialize ROS2 communications
	auto node = std::make_shared<VeloPublisher>();
	rclcpp::spin(node); // 'spin()' means run the node indefinitely until it's killed
	rclcpp::shutdown(); // End ROS2 communications
	return 0;
}
