#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LiDARSubscriber : public rclcpp::Node {
    public:
        LiDARSubscriber() : Node("lidar_sub") {
            subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>
            ("/model/diff_drive_blu/chassis_lidar/forward", 
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                topic_callback(msg);
            });
        }
    private:
        void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr message) const {
            RCLCPP_INFO(this->get_logger(), "Recieved min_ang:%lf, max_ang:%lf, range_min:%lf, range_max:%lf", 
                message->angle_min,
                message->angle_max,
                message->range_min,
                message->range_max
            );
        }
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
}; 

int main(int argc, char **argv) {
	rclcpp::init(argc, argv); // Initialize ROS2 communications
	rclcpp::spin(std::make_shared<LiDARSubscriber>()); // 'spin()' means run the node indefinitely until it's killed
	rclcpp::shutdown(); // End ROS2 communications
	return 0;
}