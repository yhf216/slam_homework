#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"  // ICP算法
#include "pcl/filters/voxel_grid.h" 

class RelocalizationNode : public rclcpp::Node
{
public:
    RelocalizationNode() : Node("relocalization_node"), tf_broadcaster_(this)
    {
        // 声明参数：实时点云话题
        this->declare_parameter<std::string>("lidar_topic","/livox/lidar/pointcloud"); //雷达话题
        //this->declare_parameter<std::string>("imu_topic","/livox/imu"); //IMU话题
        this->declare_parameter<std::string>("map_topic","/static_pointcloud_map"); //地图话题
        this->declare_parameter<std::string>("map_frame","odom_init"); //地图坐标系
        this->declare_parameter<int>("icp_iterations", 50);  // ICP迭代次数（简化计算）

        // 获取参数值
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        //std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        int icp_iter = this->get_parameter("icp_iterations").as_int();

        // 配置简易ICP算法参数
        icp_.setMaximumIterations(icp_iter);
        icp_.setTransformationEpsilon(1e-3);


        // 订阅消息
        // 地图
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic,
            10,
            std::bind(&RelocalizationNode::mapCallback, this, std::placeholders::_1)
        );
        // 雷达
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic,
            10,
            std::bind(&RelocalizationNode::lidarCallback, this, std::placeholders::_1)
        );

        // IMU
        //imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        //    imu_topic,
        //    10,
        //    std::bind(&RelocalizationNode::imuCallback, this, std::placeholders::_1)
        //);

        // 初始化轨迹发布者
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        RCLCPP_INFO(this->get_logger(), "简化版重定位节点（ICP算法）启动");
        RCLCPP_INFO(this->get_logger(), "订阅地图: %s, 雷达: %s", map_topic.c_str(), lidar_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "发布轨迹: /path（坐标系: %s）", map_frame_.c_str());

    }

private:
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (map_loaded_) return;

        pcl::fromROSMsg(*msg, *map_cloud_);
        map_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "静态地图加载完成，点数: %zu，开始等待雷达数据...", map_cloud_->size());
        icp_.setInputTarget(map_cloud_);  // 设置ICP目标点云（地图）
    }
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // from AI 待学习部分

        RCLCPP_INFO(this->get_logger(), "收到雷达点云，宽度: %d, 高度: %d", 
               msg->width, msg->height);
               
        if (!map_loaded_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "等待静态地图加载...");
            return;
        }

        // 1. 转换实时点云为PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr live_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *live_cloud);

        // 2. 简化：对实时点云降采样（减少计算量）
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_live(new pcl::PointCloud<pcl::PointXYZ>());
        downsampleCloud(live_cloud, filtered_live, 0.2);  // 体素滤波，分辨率0.2m

        // 3. ICP匹配（用上次结果作为初始猜测，加速收敛）
        icp_.setInputSource(filtered_live);
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;  // 匹配后的点云
        icp_.align(aligned_cloud, last_transform_);

        
        if (icp_.hasConverged()) {
            last_transform_ = icp_.getFinalTransformation();
            publishTrajectory(last_transform_, msg->header.stamp);
            RCLCPP_INFO(this->get_logger(), "ICP匹配成功！误差: %.4f，已发布轨迹", icp_.getFitnessScore());
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP匹配失败！当前误差: %.4f，未发布轨迹", icp_.getFitnessScore());
        }

        // 4. 若匹配成功，更新位姿并发布轨迹
        if (icp_.hasConverged()) {
            last_transform_ = icp_.getFinalTransformation();  // 保存变换矩阵
            publishTrajectory(last_transform_, msg->header.stamp);  // 发布轨迹
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                               "ICP匹配成功，误差: %.4f", icp_.getFitnessScore());
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP匹配失败，使用上次位姿");
        }
    }

    //void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    //{

    //}

    // from AI 待学习部分
    // 点云降采样（简化计算）
    void downsampleCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
        float leaf_size)
    {
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel.filter(*output);
    }

    // from AI 待学习部分
    // 发布轨迹和TF变换
    void publishTrajectory(const Eigen::Matrix4f& transform, const rclcpp::Time& stamp)
    {
        // 1. 从变换矩阵提取位姿
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_;
        // 位置（x,y,z）
        pose.pose.position.x = transform(0, 3);
        pose.pose.position.y = transform(1, 3);
        pose.pose.position.z = transform(2, 3);
        // 姿态（从旋转矩阵转四元数）
        Eigen::Quaternionf quat(transform.block<3,3>(0,0));
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        // 2. 更新轨迹
        path_.header = pose.header;
        path_.poses.push_back(pose);
        if (path_.poses.size() > 2000) {  // 限制轨迹长度，避免内存溢出
            path_.poses.erase(path_.poses.begin());
        }
        path_pub_->publish(path_);

        // 3. 发布TF（odom_init → base_link），确保RViz显示正确
        geometry_msgs::msg::TransformStamped tf;
        tf.header = pose.header;
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = pose.pose.position.x;
        tf.transform.translation.y = pose.pose.position.y;
        tf.transform.translation.z = pose.pose.position.z;
        tf.transform.rotation = pose.pose.orientation;
        tf_broadcaster_.sendTransform(tf);
    }

    // 成员变量
    std::string map_frame_;
    bool map_loaded_ = false;
    Eigen::Matrix4f last_transform_ = Eigen::Matrix4f::Identity();  // 上次变换矩阵（初始为单位矩阵）

    // 点云与ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;  // ICP匹配器

    // 订阅与发布
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 轨迹数据
    nav_msgs::msg::Path path_;
};

int main(int argc, char**argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}