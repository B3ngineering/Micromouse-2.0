#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover") {
        // Create a publisher for /cmd_vel
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 10, 
            std::bind(&MouseMover::laser_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Set a timer to periodically call the move_robot function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MouseMover::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized.");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Define a safe distance to the nearest obstacle
        const float safe_distance = 0.5;
        wall_detected_ = false;

        // Check if the robot is too close to an obstacle
        for (const auto &range : msg->ranges) {
            RCLCPP_INFO(this->get_logger(), "Range: %f", range);
            if (range < safe_distance) {
                wall_detected_ = true;
                break;
            }
        }
    }

    void move_mouse() {
        auto twist = geometry_msgs::msg::Twist();

        // If a wall is detected, stop the robot
        if (wall_detected_) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.5;
            RCLCPP_INFO(this->get_logger(), "Wall detected! Turning.");
        } else {
            twist.linear.x = 0.5;
            twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "No wall detected. Moving forward.");
        }

        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool wall_detected_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseMover>());
    rclcpp::shutdown();
    return 0;
}
