#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/passthrough.h"  // 用于z轴过滤
#include <filesystem> 

namespace fs = std::filesystem;

//FIXME:删除没用的功能
class PCDSaverNode : public rclcpp::Node
{
public:
    PCDSaverNode() : Node("pcd_saver_node"),
                     map_cloud_pcl_(new pcl::PointCloud<pcl::PointXYZ>()),
                     accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>())
    {
        // 声明参数：订阅点云话题名称
        this->declare_parameter<std::string>("pointcloud_topic","/cloud_registered");
        // 声明参数：保存PCD文件的目录
        this->declare_parameter<std::string>("save_path","/home/t1ppler/slam_homework/assets/pcd_maps");//FIXME: 改为相对路径

        // 获取参数值
        std::string topic_name = this->get_parameter("pointcloud_topic").as_string();
        save_path_ = this->get_parameter("save_path").as_string();

        // 处理保存路径
        //if (save_path_.starts_with("~")) {
        //    save_path_.replace(0, 1, getenv("HOME"));
        //}
        // 若路径不存在，自动创建
        if (!fs::exists(save_path_)) {
            fs::create_directories(save_path_);
            RCLCPP_INFO(this->get_logger(), "创建保存目录: %s", save_path_.c_str());
        }

        // 订阅点云话题
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic_name,
            10,
            std::bind(&PCDSaverNode::pointcloudCallback, this, std::placeholders::_1)
        );

        // 创建保存服务
        save_service_ = this->create_service<std_srvs::srv::Empty>(
            "/save_pcd",
            std::bind(&PCDSaverNode::savePCDCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // 初始化地图发布者（话题名：/static_pointcloud_map）
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/static_pointcloud_map", 10);

        publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PCDSaverNode::publishMapCloud, this)
        );

        RCLCPP_INFO(this->get_logger(), "PCD保存节点已启动");
        RCLCPP_INFO(this->get_logger(), "订阅点云话题: %s", topic_name.c_str());
        RCLCPP_INFO(this->get_logger(), "保存路径: %s", save_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "调用服务: ros2 service call /save_pcd std_srvs/srv/Empty \"{}\" 触发保存");        
        RCLCPP_INFO(this->get_logger(), "已初始化/static_pointcloud_map发布（1Hz），坐标系: odom_init");
        RCLCPP_INFO(this->get_logger(), "开始累积点云（合并所有接收帧）...");  // 新增：提示累积逻辑
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_cloud_ = msg;

        // 将ROS点云消息转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // 累积点云
        *accumulated_cloud_ += *pcl_cloud;

    }

    void savePCDCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response)
    {
        (void)request;
        (void)response;

        if (accumulated_cloud_->empty()) {  // 检查累积点云是否为空
            RCLCPP_WARN(this->get_logger(), "累积点云为空，无法保存");
            return;
        }

        // 转换为PCL点云格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*latest_cloud_, *pcl_cloud);

        // 生成带时间戳的文件名
        auto now = this->get_clock()->now();
        std::string timestamp = std::to_string(now.seconds()) + "_" + std::to_string(now.nanoseconds() / 1000000);
        std::string filename = save_path_ + "/cloud_" + timestamp + ".pcd";

        // 保存累积的完整点云
        if (pcl::io::savePCDFileBinary(filename, *accumulated_cloud_) == 0) {  //保存累积点云
            RCLCPP_INFO(this->get_logger(), "完整地图保存成功: %s（总点数: %zu）", 
                       filename.c_str(), accumulated_cloud_->size());
        } else {
            RCLCPP_ERROR(this->get_logger(), "完整地图保存失败: %s", filename.c_str());
        }
    }

    void filterZAxis()
    {
        if(accumulated_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "累积点云为空，无法进行Z轴过滤");
            map_cloud_ros_= nullptr;
            return;
        }

        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(accumulated_cloud_);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-std::numeric_limits<float>::max(), 1.5);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pass.filter(*filtered_cloud);

        map_cloud_ros_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*filtered_cloud, *map_cloud_ros_);
        map_cloud_ros_->header.frame_id = "odom_init";
    }

    void publishMapCloud()
    {
        filterZAxis();  // 先进行Z轴过滤

        if (!map_cloud_ros_ || map_cloud_ros_->data.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "无有效累积点云可发布");
            return;
        }

        map_cloud_ros_->header.stamp = this->get_clock()->now();
        map_pub_->publish(*map_cloud_ros_);
        RCLCPP_INFO(this->get_logger(), "发布静态点云地图，点数: %zu", map_cloud_ros_->data.size() / sizeof(float) / 3);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_service_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr map_cloud_ros_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_pcl_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    std::string save_path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PCDSaverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//ros2 service call /save_pcd std_srvs/srv/Empty "{}" //触发保存