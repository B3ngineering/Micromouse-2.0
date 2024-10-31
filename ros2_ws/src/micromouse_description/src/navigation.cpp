#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

using namespace std;

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover"), state_(State::MOVING_FORWARD), initial_yaw_(0.0), target_yaw_(0.0) {
        // Create a publisher for /cmd_vel
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 10, 
            std::bind(&MouseMover::laser_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Create a subscriber for /odom
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&MouseMover::odom_callback, this, std::placeholders::_1));

        // Set a timer to periodically call the move_robot function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&MouseMover::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized.");
    }

private:
    enum class State {
        MOVING_FORWARD,
        DETECTING_WALL,
        TURNING
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Only check for walls if in the DETECTING_WALL state
        if (state_ == State::DETECTING_WALL) {
            const float safe_distance = 0.5;
            wall_detected_ = false;

            for (const auto &range : msg->ranges) {
                if (range < safe_distance) {
                    wall_detected_ = true;
                    break;
                }
            }

            // Transition to TURNING state if a wall is detected
            if (wall_detected_) {
                state_ = State::TURNING;
                initial_yaw_ = current_yaw_;
                target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2); // Set target to 90 degrees
                RCLCPP_INFO(this->get_logger(), "Wall detected! Preparing to turn.");
            } else {
                state_ = State::MOVING_FORWARD; // No wall, continue moving forward
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Always update yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        current_yaw_ = yaw;
    }

    void move_mouse() {
        auto twist = geometry_msgs::msg::Twist();

        switch (state_) {
            case State::MOVING_FORWARD:
                twist.linear.x = 0.5;
                twist.angular.z = 0.0;
                state_ = State::DETECTING_WALL; // Check for walls after moving forward
                RCLCPP_INFO(this->get_logger(), "Moving forward.");
                break;

            case State::TURNING: {
                double angle_difference = normalize_angle(target_yaw_ - current_yaw_);
                const double tolerance = 0.01;

                if (fabs(angle_difference) > tolerance) {
                    // Proportional control to turn more smoothly
                    twist.angular.z = 1.0 * (angle_difference > 0 ? 1.0 : -1.0);
                    twist.linear.x = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Turning.");
                } else {
                    twist.angular.z = 0.0;
                    state_ = State::MOVING_FORWARD; // After turning, continue moving forward
                    RCLCPP_INFO(this->get_logger(), "Turn complete. Moving forward.");
                }
                break;
            }

            case State::DETECTING_WALL:
                // Wait for laser_callback to set wall_detected_ and update the state
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                break;
        }

        publisher_->publish(twist);
    }

    double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    bool wall_detected_;
    double initial_yaw_;
    double current_yaw_;
    double target_yaw_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseMover>());
    rclcpp::shutdown();
    return 0;
}
