#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread> // Include for sleep functionality

using namespace std;

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover"), state_(State::MOVING_FORWARD), initial_yaw_(0.0), target_yaw_(0.0), distance_traveled_(0.0) {
        // Subscribe to Lidar data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 5, 
            std::bind(&MouseMover::laser_callback, this, std::placeholders::_1));

        // Subscribe to Odometry data
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 5, 
            std::bind(&MouseMover::odom_callback, this, std::placeholders::_1));
        
        // Create a publisher for the robot's velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Set a timer to periodically call the move_robot function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&MouseMover::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized.");
    }

private:
    enum class State {
        MOVING_FORWARD,
        DETECTING_WALL,
        TURNING,
        REFINING_TURN
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_ == State::DETECTING_WALL) {
            const float safe_distance = 0.75;
            wall_detected_ = false;

            float front_distance = msg->ranges[0];
            float right_distance = msg->ranges[3];
            float left_distance = msg->ranges[1];
            RCLCPP_INFO(this->get_logger(), "Right distance: %f", right_distance);
            RCLCPP_INFO(this->get_logger(), "Front distance: %f", front_distance);
            RCLCPP_INFO(this->get_logger(), "Left distance: %f", left_distance);

            if (front_distance < safe_distance) {
                wall_detected_ = true;
            }

            if (right_distance > 1) {
                state_ = State::TURNING;
                initial_yaw_ = current_yaw_;
                target_yaw_ = normalize_angle(initial_yaw_ - M_PI / 2);

                RCLCPP_INFO(this->get_logger(), "No wall on the right. Preparing to turn right.");
            } else if (wall_detected_) {
                if (left_distance < 0.6) {
                    state_ = State::TURNING;
                    initial_yaw_ = current_yaw_;
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI);
                    RCLCPP_INFO(this->get_logger(), "Wall detected in front. Preparing to turn left.");
                } else {
                    state_ = State::TURNING;
                    initial_yaw_ = current_yaw_;
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
                    RCLCPP_INFO(this->get_logger(), "Wall detected in front. Preparing to turn left.");
                }
            } else {
                state_ = State::MOVING_FORWARD;
                distance_traveled_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "Wall detected on the right. Moving forward.");
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

        double new_x = msg->pose.pose.position.x;
        double new_y = msg->pose.pose.position.y;

        distance_traveled_ += sqrt(pow(new_x - current_x_, 2) + pow(new_y - current_y_, 2));

        current_x_ = new_x;
        current_y_ = new_y;
    }

    void move_mouse() {
        auto twist = geometry_msgs::msg::Twist();

        // Check if the robot has reached the goal
        if ((18.5 <= current_x_ && current_x_ <= 19.5) && (18.5 <= current_y_ && current_y_ <= 19.5)) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            rclcpp::shutdown();
        }

        switch (state_) {
            case State::MOVING_FORWARD: {
                if (distance_traveled_ < 0.49) {
                    twist.linear.x = 0.5;
                    twist.angular.z = 0.0;
                } else {
                    twist.linear.x = 0.0;
                    state_ = State::DETECTING_WALL;
                    RCLCPP_INFO(this->get_logger(), "0.5 meter traveled. Checking for walls.");
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // Sleep for 100 milliseconds
                }
                
                break;
            }

            case State::TURNING: {
                double angle_difference = normalize_angle(target_yaw_ - current_yaw_);
                const double tolerance = 0.01;

                // Proportional control to turn more smoothly
                if (fabs(angle_difference) > tolerance) {
                    twist.angular.z = 1.0 * (angle_difference > 0 ? 1.0 : -1.0);
                    twist.linear.x = 0.0;
                    // RCLCPP_INFO(this->get_logger(), "Turning.");
                } else {
                    twist.angular.z = 0.0;
                    state_ = State::REFINING_TURN; 
                    RCLCPP_INFO(this->get_logger(), "Turn complete. Refining turn.");
                }
                break;
            }

            case State::REFINING_TURN: {
                double angle_difference = normalize_angle(target_yaw_ - current_yaw_);
                const double fine_tolerance = 0.001; 

                // Fine-tune the turn for precision
                if (fabs(angle_difference) > fine_tolerance) {
                    twist.angular.z = 0.2 * (angle_difference > 0 ? 1.0 : -1.0);
                    twist.linear.x = 0.0;
                    // RCLCPP_INFO(this->get_logger(), "Refining turn.");
                } else {
                    twist.angular.z = 0.0;
                    state_ = State::MOVING_FORWARD;
                    distance_traveled_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Refinement complete. Moving forward.");
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
    double current_x_;
    double current_y_;
    double distance_traveled_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MouseMover>());
    rclcpp::shutdown();
    return 0;
}
