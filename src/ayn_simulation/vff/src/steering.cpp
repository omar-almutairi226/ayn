#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

using namespace std::chrono_literals;
class SteeringAlgorithm : public rclcpp::Node
{
public:
    SteeringAlgorithm() : Node("steering_algorithm")
    {
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SteeringAlgorithm::scanCallback, this, std::placeholders::_1));

        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_vff", 10);

        timer_ = this->create_wall_timer(
            500ms, std::bind(&SteeringAlgorithm::publishCmd, this));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto min_range = msg->range_max;
        int min_index = -1;
        
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < min_range)
            {
                min_range = msg->ranges[i];
                min_index = i;
            }
        }

        if (min_index != -1)
        {
            double angle = msg->angle_min + min_index * msg->angle_increment;
            if (min_range < SAFE_DISTANCE)
            {
                if (angle > 0)
                {
                    steering_angle_ = -MAX_TURN_ANGLE;
                }
                else
                {
                    steering_angle_ = MAX_TURN_ANGLE;
                }
            }
            else
            {
                steering_angle_ = 0.0;
            }
        }
    }

    void publishCmd()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.angular.z = steering_angle_;
        cmd_msg.linear.x = FORWARD_SPEED;
        cmd_publisher_->publish(cmd_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double steering_angle_ = 0.0;
    const double SAFE_DISTANCE = 1.0;
    const double MAX_TURN_ANGLE = 0.5;
    const double FORWARD_SPEED = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteeringAlgorithm>());
    rclcpp::shutdown();
    return 0;
}
