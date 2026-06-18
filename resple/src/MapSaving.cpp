#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <mutex>

class MapSaving : public rclcpp::Node
{
public:
    MapSaving() : Node("MapSaving")
    {
        pcd_save_path = this->declare_parameter<std::string>("pcd_save_path", "/tmp/global_map.pcd");

        sub_global_map = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 200,
            std::bind(&MapSaving::globalMapCallback, this, std::placeholders::_1));

        srv_save_map = this->create_service<std_srvs::srv::Empty>(
            "save_map_node",
            std::bind(&MapSaving::savePCDCallback, this, std::placeholders::_1, std::placeholders::_2));

        accumulated_map.reset(new pcl::PointCloud<pcl::PointXYZI>());

        RCLCPP_INFO(this->get_logger(), "MapSaving node started, subscribing to 'global_map'.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_global_map;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr srv_save_map;
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map;
    std::mutex mtx_map;
    std::string pcd_save_path;

    void globalMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        std::lock_guard<std::mutex> lock(mtx_map);
        *accumulated_map += *cloud;
    }

    void savePCDCallback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                         std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        pcl::PointCloud<pcl::PointXYZI> map_copy;
        {
            std::lock_guard<std::mutex> lock(mtx_map);
            map_copy = *accumulated_map;
        }
        pcl::io::savePCDFileBinary(pcd_save_path, map_copy);
        RCLCPP_INFO(this->get_logger(), "Saved map to %s (%zu points)",
            pcd_save_path.c_str(), map_copy.size());
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapSaving>());
    rclcpp::shutdown();
    return 0;
}
