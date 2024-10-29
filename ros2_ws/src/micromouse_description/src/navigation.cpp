#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>


using namespace std;

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover"), wall_detected_(false), turning_(false), initial_yaw_(0.0) {
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
            std::chrono::milliseconds(100), std::bind(&MouseMover::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized.");
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (turning_) {
            return;
        }
        // Define a safe distance to the nearest obstacle
        const float safe_distance = 0.5;
        wall_detected_ = false;

        // Check if the robot is too close to an obstacle
        for (const auto &range : msg->ranges) {
            // RCLCPP_INFO(this->get_logger(), "Range: %f", range);
            if (range < safe_distance) {
                wall_detected_ = true;
                break;
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (turning_) {
            // Get the current yaw angle
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
    }

    void move_mouse() {
        auto twist = geometry_msgs::msg::Twist();

        // If a wall is detected, stop the robot
        if (wall_detected_ && !turning_) {
            turning_ = true;
            initial_yaw_ = current_yaw_;
            target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
            twist.linear.x = 0.0;
            twist.angular.z = -1.0;
            RCLCPP_INFO(this->get_logger(), "Wall detected! Turning.");
        } else if (turning_) {
            // Continue turning until the robot has turned 90 degrees
            if (abs(normalize_angle(current_yaw_ - target_yaw_)) < 0.1) {
                turning_ = false;
                wall_detected_ = false;
                twist.angular.z = 0.0;
                initial_yaw_ = 0.0;
                current_yaw_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Finished turning. Moving forward.");
                this_thread::sleep_for(chrono::milliseconds(500));
            } else {
                twist.angular.z = -1.0;
            }
        } else {
            twist.linear.x = 0.7;
            twist.angular.z = 0.0;
            RCLCPP_INFO(this->get_logger(), "No wall detected. Moving forward.");
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

    bool wall_detected_;
    bool turning_;
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
