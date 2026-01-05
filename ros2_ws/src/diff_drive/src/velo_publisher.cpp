#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VeloPublisher : public rclcpp::Node {
	public:
		VeloPublisher() : Node("velocity_pub") {

			// This create_publisher member function takes Twist as a template parameter and sets it
			// as the message type. The topic name specifies the communication channel that this
			// node will communicate on. The second parameter is the queue size, mostly important 
			// when the publisher publishes faster than the subscriber can recieve.

			publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("topic_vel", 1);
			timer_ = this->create_wall_timer(
				std::chrono::milliseconds(500),
				std::bind(&VeloPublisher::velo_callback, this)
			);
		}

	private:
		void velo_callback() {
			// Twist ia a message type for expressing velocity linear/angular
			auto message = geometry_msgs::msg::Twist(); 
			message.linear.x = 0.5;
			message.linear.y = 0;
			message.linear.z = 0;
			message.angular.x = 0;
			message.angular.y = 0;
			message.angular.z = 0;
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
