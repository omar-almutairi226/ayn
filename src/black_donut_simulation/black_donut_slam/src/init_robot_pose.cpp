#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class InitPose : public rclcpp::Node
{
  public:
    InitPose()
    : Node("init_pubilsher"), count_(0)
    {
      publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&InitPose::timer_callback, this));
    }

  private:
    void timer_callback()
    {
    
        auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        
        msg.header.frame_id = "map";
        tf2::Quaternion q; q.setRPY(0,0,0);
        geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
        msg.pose.pose.position.x = 0.0;
        msg.pose.pose.position.y = 0.0;
        msg.pose.pose.orientation = q_msg;
        msg.pose.covariance = { 0.25,0,0,0,0,0,
                                0,0.25,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0,
                                0,0,0,0,0,0.25};

        publisher_->publish(msg);
        if(count_== 10) { rclcpp::shutdown(); RCLCPP_INFO(get_logger(),"OK i will shutdown."); }
        RCLCPP_INFO(get_logger(),"Publish initial pose");
        count_++;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InitPose>());
  rclcpp::shutdown();
  return 0;
}