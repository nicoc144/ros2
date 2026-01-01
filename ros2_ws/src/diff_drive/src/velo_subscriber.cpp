#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
using std::placeholders::_1;

class VeloSubscriber : public rclcpp::Node {
	public:
		VeloSubscriber() : Node("velocity_sub") {

			// This create_subscription member function takes Twist as a template parameter and sets it
			// as the message type. The topic name specifies the communication channel that this
			// node will communicate on. The second parameter specifies the number of unread messages
			// to keep in queue if messages arrive faster than the callback can execute. 

			// The callback function is the third parameter and is called automatically every time a Twist 
			// message arrives to the topic 'subscriber_' is subscribed to. Lambdas are required for 
			// passing a function into the create_subscriber() member function as a parameter. Using a
			// normal function pointer would be problematic since the function needs to know that 'this'
			// node is calling it.

			subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("topic_vel", 1, 
			[this](geometry_msgs::msg::Twist::SharedPtr msg) {
				topic_callback(msg); // Call the private member callback function
			});
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