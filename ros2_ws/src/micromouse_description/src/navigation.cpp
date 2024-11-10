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

using namespace std;

class MouseMover : public rclcpp::Node {
public:
    MouseMover() : Node("Mouse_Mover"), state_(State::MOVING_FORWARD), initial_yaw_(0.0), target_yaw_(0.0), distance_traveled_(0.0) {
        // Subscribe to Lidar data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 5, 
            bind(&MouseMover::laser_callback, this, placeholders::_1));

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

        initialize_maze();
    }

private:
    enum class State {
        MOVING_FORWARD,
        DETECTING_WALL,
        TURNING,
        REFINING_TURN
    };

    // Maze information
    const int MAZE_SIZE_ = 20;
    vector<vector<int>> maze_;
    int goal_x_ = 19;
    int goal_y_ = 19;

    void initialize_maze() {
        // Resize the maze grid to MAZE_SIZE x MAZE_SIZE and fill with Manhattan distances
        maze_.resize(MAZE_SIZE_, vector<int>(MAZE_SIZE_, 0));

        for (int x = 0; x < MAZE_SIZE_; ++x) {
            for (int y = 0; y < MAZE_SIZE_; ++y) {
                // Set each cell to its Manhattan distance from the goal cell (19, 19)
                maze_[x][y] = 0;
            }
        }

        // Set the values circling the maze to 1
        for (int i = 0; i < MAZE_SIZE_; ++i) {
            maze_[0][i] = 1; // Top row
            maze_[MAZE_SIZE_ - 1][i] = 1; // Bottom row
            maze_[i][0] = 1; // Left column
            maze_[i][MAZE_SIZE_ - 1] = 1; // Right column
        }

        // Print the maze with its values
        for (int x = 0; x < MAZE_SIZE_; ++x) {
            for (int y = 0; y < MAZE_SIZE_; ++y) {
                cout << maze_[x][y] << " ";
            }
            cout << endl;
        }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_ == State::DETECTING_WALL) {
            const float safe_distance = 0.6;
            int x = static_cast<int>(round(current_x_));
            int y = static_cast<int>(round(current_y_));
            RCLCPP_INFO(this->get_logger(), "Current cell: (%d, %d)", x, y);
            traveled_path_.push_back({x, y});

            // Determine the direction the robot is facing based on yaw
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

            if (msg->ranges[0] < safe_distance) {
                maze_[x + forward_x][y + forward_y] = 1; // Front wall
            }
            if (msg->ranges[1] < safe_distance) {
                maze_[x + left_x][y + left_y] = 1; // Left wall
            }
            if (msg->ranges[3] < safe_distance) {
                maze_[x + right_x][y + right_y] = 1; // Right wall
            }
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
        if (traveled_path_.size() > 3 && (0.5 <= current_x_ && current_x_ <= 1.5) && (0.5 <= current_y_ && current_y_ <= 1.5)) {
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
            publisher_->publish(twist);
            RCLCPP_INFO(this->get_logger(), "Maze mapping successful!");
            for (const auto& cell : traveled_path_) {
                maze_[cell.first][cell.second] = 0;
            }
            ofstream maze_file("../src/maze.bin", ios::binary);
            for (const auto& row : maze_) {
                maze_file.write(reinterpret_cast<const char*>(row.data()), row.size() * sizeof(int));
            }
            maze_file.close();
            RCLCPP_INFO(this->get_logger(), "Maze saved to maze.bin.");
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
                    this_thread::sleep_for(chrono::milliseconds(1000)); // Sleep for 100 milliseconds
                    // Print the maze with its values
                    for (int x = 0; x < MAZE_SIZE_; ++x) {
                        for (int y = 0; y < MAZE_SIZE_; ++y) {
                            cout << maze_[x][y] << " ";
                        }
                        cout << endl;
                    }
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
    vector<pair<int, int>> traveled_path_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<MouseMover>());
    rclcpp::shutdown();
    return 0;
}
