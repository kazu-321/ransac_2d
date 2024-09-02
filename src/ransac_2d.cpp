#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class RANSAC : public rclcpp::Node{
public:
    RANSAC() : Node("ransac_localization") {
        pose_pub_     = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
        occupancy_sub_= this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10,
                        std::bind(&RANSAC::occupancy_callback, this, std::placeholders::_1));
        scan_sub_     = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), 
                        std::bind(&RANSAC::scan_callback, this, std::placeholders::_1));
        iter = this->declare_parameter("iter", 100);
    }

private:
    struct Point {
        double x, y;
    };

    void occupancy_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_points.clear();
        for(int i = 0; i < msg->data.size(); i++) {
            if(msg->data[i] == 100) {
                Point p;
                p.x = msg->info.origin.position.x + (i % msg->info.width) * msg->info.resolution;
                p.y = msg->info.origin.position.y + (i / msg->info.width) * msg->info.resolution;
                map_points.push_back(p);
            }
        }
    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_points.clear();
        for(int i = 0; i < msg->ranges.size(); i++) {
            if(msg->ranges[i] < msg->range_max) {
                Point p;
                p.x = msg->ranges[i] * cos(msg->angle_min + i * msg->angle_increment);
                p.y = msg->ranges[i] * sin(msg->angle_min + i * msg->angle_increment);
                scan_points.push_back(p);
            }
        }
        rclcpp::Time start = this->now();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    std::vector<Point> map_points;
    std::vector<Point> scan_points;
    int iter=100;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RANSAC>());
    rclcpp::shutdown();
    return 0;
}
