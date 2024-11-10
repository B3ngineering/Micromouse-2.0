#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <climits>
#include <fstream>
#include <cmath>

using namespace std;

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover"), state_(State::MOVING_FORWARD), initial_yaw_(0.0), target_yaw_(0.0), distance_traveled_(0.0) {

        // Subscribe to Odometry data
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 5, 
            bind(&MouseMover::odom_callback, this, placeholders::_1));
        
        // Create a publisher for the robot's velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Set a timer to periodically call the move_robot function
        timer_ = this->create_wall_timer(
            chrono::milliseconds(10), bind(&MouseMover::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Navigation node initialized.");

        read_path_from_file();
    }

private:
    enum class State {
        MOVING_FORWARD,
        TURNING,
        REFINING_TURN
    };

    void read_path_from_file() {
        ifstream infile("src/micromouse_description/src/path.bin", ios::binary);
        if (!infile) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open path.bin");
            return;
        }

        int x, y;
        // Remove the first value from the binary before reading
        int dummy;
        infile.read(reinterpret_cast<char*>(&dummy), sizeof(dummy));
        while (infile.read(reinterpret_cast<char*>(&x), sizeof(x)) &&
               infile.read(reinterpret_cast<char*>(&y), sizeof(y))) {
            path_.emplace_back(x, y);
        }

        infile.close();
        if (!path_.empty()) {
            path_.erase(path_.begin());
        }
        RCLCPP_INFO(this->get_logger(), "Path loaded from path.bin");
    }

    void handle_next() {

        // Check if there is a wall in front of the robot
        wall_detected_ = false;

        int forward_x = 0, forward_y = 0;
        int left_x = 0, left_y = 0;
        int right_x = 0, right_y = 0;
        string direction;
        if (fabs(current_yaw_ - 0.0) < 0.1) { // Facing East
                forward_x = 1; forward_y = 0;
                left_x = 0; left_y = 1;
                right_x = 0; right_y = -1;
                direction = "East";
            } else if (fabs(current_yaw_ - M_PI_2) < 0.1) { // Facing North
                forward_x = 0; forward_y = 1;
                left_x = -1; left_y = 0;
                right_x = 1; right_y = 0;
                direction = "North";
            } else if (fabs(current_yaw_ + M_PI_2) < 0.1) { // Facing South
                forward_x = 0; forward_y = -1;
                left_x = 1; left_y = 0;
                right_x = -1; right_y = 0;
                direction = "South";
            } else if (fabs(current_yaw_ - M_PI) < 0.1 || fabs(current_yaw_ + M_PI) < 0.1) { // Facing West
                forward_x = -1; forward_y = 0;
                left_x = 0; left_y = -1;
                right_x = 0; right_y = 1;
                direction = "West";
            }
        
        RCLCPP_INFO(this->get_logger(), "Current direction: %s", direction.c_str());

        // Determine the next direction based on the current path value
        if (!path_.empty()) {
            auto next_point = path_.front();
            path_.erase(path_.begin());

            int next_x = next_point.first;
            int next_y = next_point.second;

            RCLCPP_INFO(this->get_logger(), "Travelling to point (%d, %d)", next_x, next_y);

            int delta_x = static_cast<int>(round(next_x - current_x_));
            int delta_y = static_cast<int>(round(next_y - current_y_));

            if (delta_x > 0) {
                // Move right
                if (direction == "North") {
                    target_yaw_ = normalize_angle(initial_yaw_ - M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "South") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "East") {
                    target_yaw_ = normalize_angle(initial_yaw_);
                    state_ = State::MOVING_FORWARD;
                } else if (direction == "West") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI);
                    state_ = State::TURNING;
                }
            } else if (delta_x < 0) {
                // Move left
                if (direction == "North") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "South") {
                    target_yaw_ = normalize_angle(initial_yaw_ - M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "East") {
                    target_yaw_ = normalize_angle(initial_yaw_ - M_PI);
                    state_ = State::TURNING;
                } else if (direction == "West") {
                    target_yaw_ = normalize_angle(initial_yaw_);
                    state_ = State::MOVING_FORWARD;
                }
            } else if (delta_y > 0) {
                // Move forward
                if (direction == "North") {
                    target_yaw_ = normalize_angle(initial_yaw_);
                    state_ = State::MOVING_FORWARD;
                } else if (direction == "South") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI);
                    state_ = State::TURNING;
                } else if (direction == "East") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "West") {
                    target_yaw_ = normalize_angle(initial_yaw_ - M_PI / 2);
                    state_ = State::TURNING;
                }
                
            } else if (delta_y < 0) {
                // Move backward
                if (direction == "North") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI);
                    state_ = State::TURNING;
                } else if (direction == "South") {
                    target_yaw_ = normalize_angle(initial_yaw_);
                    state_ = State::MOVING_FORWARD;
                } else if (direction == "East") {
                    target_yaw_ = normalize_angle(initial_yaw_ - M_PI / 2);
                    state_ = State::TURNING;
                } else if (direction == "West") {
                    target_yaw_ = normalize_angle(initial_yaw_ + M_PI / 2);
                    state_ = State::TURNING;
                }

            }
            distance_traveled_ = 0.0;
            initial_yaw_ = current_yaw_;

        } else {
            state_ = State::MOVING_FORWARD;
            distance_traveled_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Path completed. Moving forward.");
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
        if ((19 <= current_x_ && current_x_ <= 19.5) && (19 <= current_y_ && current_y_ <= 19.5)) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Maze traversed successfully!");

            rclcpp::shutdown();
        }

        switch (state_) {
            case State::MOVING_FORWARD: {
                if (distance_traveled_ < 0.99) {
                    twist.linear.x = 0.5;
                    twist.angular.z = 0.0;
                } else {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    publisher_->publish(twist);
                    RCLCPP_INFO(this->get_logger(), "Robot stopped.");
                    RCLCPP_INFO(this->get_logger(), "0.5 meter traveled. Checking for walls.");
                    this_thread::sleep_for(chrono::milliseconds(1000));
                    handle_next();
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
    vector<pair<int, int>> path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MouseMover>());
    rclcpp::shutdown();
    return 0;
}
