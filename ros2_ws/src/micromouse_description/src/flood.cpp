#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <thread>
#include <vector>
#include <queue>
#include <climits>

using namespace std;

class FloodFill : public rclcpp::Node {
public:
    FloodFill() : Node("Flood_Fill"), state_(State::MOVING_FORWARD), initial_yaw_(0.0), target_yaw_(0.0), distance_traveled_(0.0) {
        // Subscribe to Lidar data
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/laser_controller/out", 5, 
            bind(&FloodFill::laser_callback, this, placeholders::_1));

        // Subscribe to Odometry data
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 5, 
            bind(&FloodFill::odom_callback, this, placeholders::_1));
        
        // Create a publisher for the robot's velocity
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Set a timer to periodically call the move_robot function
        timer_ = this->create_wall_timer(
            chrono::milliseconds(10), bind(&FloodFill::move_mouse, this));

        RCLCPP_INFO(this->get_logger(), "Flood fill node initialized.");

        initialize_maze();
    }

private:

    // Maze information
    const int MAZE_SIZE_ = 20;
    vector<vector<int>> maze_;
    int goal_x_ = 19;
    int goal_y_ = 19;

    void initialize_maze () {
        // Resize the maze grid to MAZE_SIZE x MAZE_SIZE and fill with Manhattan distances
        maze_.resize(MAZE_SIZE_, vector<int>(MAZE_SIZE_, 0));

        for (int x = 0; x < MAZE_SIZE_; ++x) {
            for (int y = 0; y < MAZE_SIZE_; ++y) {
                // Set each cell to its Manhattan distance from the goal cell (19, 19)
                maze_[x][y] = abs(goal_x_ - x) + abs(goal_y_ - y);
            }
        }
        // Print the maze with its values
        for (int x = 0; x < MAZE_SIZE_; ++x) {
            for (int y = 0; y < MAZE_SIZE_; ++y) {
            cout << maze_[x][y] << " ";
            }
            cout << endl;
        }
        
    }


    enum class State {
        MOVING_FORWARD,
        DETECTING_WALL,
        TURNING,
        REFINING_TURN
    };

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Similar logic to detect walls and decide turning direction
        // ...
        // Determine valid naighbours based on which cells are occupied
        // Check values of neighbours based on the current position

    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Similar logic to update current position and yaw
        // ...
    }

    void move_mouse() {
        auto twist = geometry_msgs::msg::Twist();

        // Implement flood fill logic to explore the maze
        // ...

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
    rclcpp::spin(std::make_shared<FloodFill>());
    rclcpp::shutdown();
    return 0;
}
