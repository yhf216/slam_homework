#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include <filesystem>

namespace fs = std::filesystem;

class MapPublisherNode : public rclcpp::Node
{
public:
    MapPublisherNode() : Node("map_publisher_node"), tf_broadcaster_(this)
    {
        // 声明参数：地图PCD文件路径
        this->declare_parameter<std::string>("map_pcd_path","/home/t1ppler/slam_homework/assets/pcd_maps/cloud_1761624483.136961_1761624483136.pcd");//FIXME: 改为相对路径

        // 声明参数： 发布频率
        this->declare_parameter<int>("publish_frequency",1);

        // 初始化智能指针
        original_cloud_ = nullptr;
        filtered_cloud_ = nullptr;

        // 获取参数值
        std::string pcd_path = this->get_parameter("map_pcd_path").as_string();
        int frequency = this->get_parameter("publish_frequency").as_int();

        // 校验pcd文件是否存在
        if (pcd_path.empty() || !fs::exists(pcd_path)) {
            RCLCPP_FATAL(this->get_logger(), "PCD文件路径无效: %s", pcd_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // 加载pcd文件
        if (!loadPCDFile(pcd_path)) {
            RCLCPP_FATAL(this->get_logger(), "加载PCD文件失败: %s", pcd_path.c_str());
            rclcpp::shutdown();
            return;
        }

        // 初始化地图发布者（话题名：/static_pointcloud_map）
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/static_pointcloud_map", 10);

        // 初始化定时器
        publish_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / frequency),
            std::bind(&MapPublisherNode::publishMap, this)
        );

        RCLCPP_INFO(this->get_logger(), "地图发布节点启动成功");
        RCLCPP_INFO(this->get_logger(), "加载的PCD文件: %s", pcd_path.c_str());
        RCLCPP_INFO(this->get_logger(), "发布话题: /static_pointcloud_map（坐标系: odom_init）");
        RCLCPP_INFO(this->get_logger(), "发布频率: %d Hz", frequency);

    }

private:
    // 加载pcd文件到PCL点云
    bool loadPCDFile(const std::string& file_path)
    {
        original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *original_cloud_) == -1) {
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "PCD文件加载成功，原始点数: %zu", original_cloud_->size());
        return true;
    }

    // 过滤z>1.5的点
    void filterZAxis()
    {
        if (original_cloud_ == nullptr || original_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "原始点云为空，无法过滤");
            filtered_cloud_ = nullptr;  
        return;
        }
        filtered_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(original_cloud_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::max(), 1.5);      
        pass.filter(*filtered_cloud_);
        RCLCPP_INFO_ONCE(this->get_logger(), "点云过滤完成，过滤后点数: %zu", filtered_cloud_->size());  
    }
    
    // 发布点云和TF
    void publishMap()
    {
        // 首次发布时过滤
        if (filtered_cloud_ == nullptr || filtered_cloud_->empty()){
            filterZAxis();
        }

        // 转换并发布
        sensor_msgs::msg::PointCloud2::SharedPtr ros_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*filtered_cloud_, *ros_cloud);
        ros_cloud->header.frame_id = "odom_init";
        ros_cloud->header.stamp = this->get_clock()->now();
        map_pub_->publish(*ros_cloud);

        // 发布odom_init->map的TF（让RViz识别odom_init坐标系）
        publishTF();
    }

    // 发布TF变换（odom_init与map重合，方便RViz显示）
    void publishTF()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = "map";  // 父坐标系（RViz常用固定坐标系）
        transform.child_frame_id = "odom_init";  // 子坐标系（点云坐标系）

        // 平移：无偏移（odom_init与map重合）
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;

        // 旋转：无旋转（四元数表示单位旋转）
        transform.transform.rotation.x = 0.0;
        transform.transform.rotation.y = 0.0;
        transform.transform.rotation.z = 0.0;
        transform.transform.rotation.w = 1.0;

        tf_broadcaster_.sendTransform(transform);
    }

    // 成员变量
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}