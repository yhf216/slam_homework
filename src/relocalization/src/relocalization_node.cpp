#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

// PCL相关
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/icp.h"
#include "pcl/features/fpfh.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/correspondence_rejection_sample_consensus.h"

// 用于IMU姿态解算
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class RelocalizationNode : public rclcpp::Node
{
public:
    RelocalizationNode() : Node("relocalization_node"), tf_broadcaster_(this)
    {
        // 声明参数
        this->declare_parameter<std::string>("lidar_topic","/livox/lidar/pointcloud"); 
        this->declare_parameter<std::string>("imu_topic","/livox/imu");
        this->declare_parameter<std::string>("map_topic","/map_for_navigation"); 
        this->declare_parameter<std::string>("map_frame","map"); 
        this->declare_parameter<int>("icp_iterations", 50);
        this->declare_parameter<double>("voxel_size", 0.1);
        this->declare_parameter<double>("ransac_threshold", 0.1);
        this->declare_parameter<double>("fpfh_radius", 0.3);
        this->declare_parameter<double>("icp_fitness_threshold", 0.5);  // ICP匹配误差阈值
        this->declare_parameter<double>("max_idle_time", 5.0);         // 最大空闲时间(秒)
        this->declare_parameter<bool>("manual_relocalize", false);     // 手动触发重定位

        // 获取参数
        std::string lidar_topic = this->get_parameter("lidar_topic").as_string();
        std::string imu_topic = this->get_parameter("imu_topic").as_string();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        voxel_size_ = this->get_parameter("voxel_size").as_double();
        ransac_threshold_ = this->get_parameter("ransac_threshold").as_double();
        icp_fitness_threshold_ = this->get_parameter("icp_fitness_threshold").as_double();
        max_idle_time_ = this->get_parameter("max_idle_time").as_double();

        // 订阅器初始化
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            map_topic,
            10,
            std::bind(&RelocalizationNode::mapCallback, this, std::placeholders::_1)
        );

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic,
            10,
            std::bind(&RelocalizationNode::lidarCallback, this, std::placeholders::_1)
        );

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            10,
            std::bind(&RelocalizationNode::imuCallback, this, std::placeholders::_1)
        );

        // 参数变化回调（用于手动触发重定位）
        manual_reloc_param_sub_ = this->add_on_set_parameters_callback(
            std::bind(&RelocalizationNode::paramCallback, this, std::placeholders::_1)
        );

        // 发布器初始化
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        
        // 初始化位姿
        last_pose_.setIdentity();
        imu_pose_.setIdentity();
        is_initialized_ = false;
        last_successful_time_ = this->get_clock()->now();
        trigger_relocalization_ = false;

        RCLCPP_INFO(this->get_logger(), "激光雷达-IMU融合重定位节点启动");
        RCLCPP_INFO(this->get_logger(), "发布轨迹: /path（坐标系: %s）", map_frame_.c_str());
    }

private:
    // 参数变化回调（处理手动重定位触发）
    rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : params) {
            if (param.get_name() == "manual_relocalize" && param.as_bool()) {
                RCLCPP_INFO(this->get_logger(), "收到手动重定位指令");
                trigger_relocalization_ = true;
                // 重置参数为false，方便下次触发
                this->set_parameter(rclcpp::Parameter("manual_relocalize", false));
            }
        }
        return result;
    }

    // IMU数据处理：获取姿态和角速度
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // 从四元数获取旋转矩阵
        Eigen::Quaternionf quat(
            msg->orientation.w,
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z);
        Eigen::Matrix3f rot = quat.toRotationMatrix();

        // 存储IMU姿态（仅旋转部分，位置由点云提供）
        imu_pose_.block<3,3>(0,0) = rot;
        
        // 条件1：检测剧烈运动（碰撞判断）触发重定位
        Eigen::Vector3f accel(msg->linear_acceleration.x, 
                             msg->linear_acceleration.y, 
                             msg->linear_acceleration.z);
        if (accel.norm() > 15.0f) {  // 加速度阈值（约1.5g）
            RCLCPP_WARN(this->get_logger(), "检测到剧烈运动，触发重定位");
            trigger_relocalization_ = true;
        }
    }
    
    // 地图点云处理
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (map_loaded_) return;
        
        // 转换为PCL点云并预处理
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *temp_cloud);
        pcl::PointCloud<pcl::PointNormal>::Ptr raw_map(new pcl::PointCloud<pcl::PointNormal>());
        pcl::copyPointCloud(*temp_cloud, *raw_map);  // 转换为PointNormal

        filterPointCloud(raw_map, filtered_map_);  // 下采样+去噪
        computeFPFHFeatures(filtered_map_, map_features_, map_tree_);  // 计算FPFH特征

        map_loaded_ = true;
        RCLCPP_INFO(this->get_logger(), "地图加载完成（点数: %zu）", filtered_map_->size());
    }

    // 实时雷达点云处理
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!map_loaded_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "等待地图加载...");
            return;
        }

        // 转换并预处理实时点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *temp_cloud);
        pcl::PointCloud<pcl::PointNormal>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointNormal>());
        pcl::copyPointCloud(*temp_cloud, *raw_cloud);
        pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointNormal>());
        filterPointCloud(raw_cloud, filtered_cloud);

        // 检查重定位触发条件
        checkRelocalizationConditions(msg->header.stamp);

        // 初始定位或触发重定位时执行粗匹配
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        if (!is_initialized_ || trigger_relocalization_) {
            RCLCPP_INFO(this->get_logger(), "执行粗匹配...");
            initial_guess = coarseRegistration(filtered_cloud);
            trigger_relocalization_ = false;
            is_initialized_ = true;
            last_successful_time_ = this->get_clock()->now();  // 更新成功时间
        } else {
            // 正常模式：用IMU姿态预测作为初始猜测
            initial_guess = last_pose_;
            initial_guess.block<3,3>(0,0) = imu_pose_.block<3,3>(0,0);  // 用IMU旋转更新
        }

        // 精匹配：Point-to-Plane ICP
        Eigen::Matrix4f final_pose = fineRegistration(filtered_cloud, initial_guess);

        // 融合IMU与ICP结果（简单加权融合）
        Eigen::Matrix4f fused_pose = fusePoses(final_pose, imu_pose_);

        // 发布轨迹
        publishTrajectory(fused_pose, msg->header.stamp);
        last_pose_ = fused_pose;
    }

    // 重定位条件检查
    void checkRelocalizationConditions(const rclcpp::Time& current_time)
    {
        // 条件2：ICP匹配误差过大（在fineRegistration中更新）
        if (last_icp_fitness_ > icp_fitness_threshold_) {
            RCLCPP_WARN(this->get_logger(), "ICP匹配误差过大(%.4f > %.4f)，触发重定位",
                      last_icp_fitness_, icp_fitness_threshold_);
            trigger_relocalization_ = true;
        }

        // 条件3：长时间未获得有效定位
        auto time_diff = current_time.seconds() - last_successful_time_.seconds();
        if (time_diff > max_idle_time_) {
            RCLCPP_WARN(this->get_logger(), "长时间未更新有效位姿(%.1fs > %.1fs)，触发重定位",
                      time_diff, max_idle_time_);
            trigger_relocalization_ = true;
        }
    }

    // 点云预处理：下采样+离群点去除
    void filterPointCloud(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& input,
        pcl::PointCloud<pcl::PointNormal>::Ptr& output)
    {
        // 体素滤波下采样
        pcl::VoxelGrid<pcl::PointNormal> voxel;
        voxel.setInputCloud(input);
        voxel.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
        voxel.filter(*output);

        // 统计滤波去噪
        pcl::StatisticalOutlierRemoval<pcl::PointNormal> sor;
        sor.setInputCloud(output);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*output);
    }

    // 计算FPFH特征
    void computeFPFHFeatures(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr& features,
        pcl::search::KdTree<pcl::PointNormal>::Ptr& tree) 
    {
        tree->setInputCloud(cloud);

        // 计算法向量（存储到cloud的normal成员）
        pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(10);
        ne.compute(*cloud);  // 直接更新输入点云的法向量

        // 计算FPFH特征
        pcl::FPFHEstimation<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(cloud);  // 使用点云自身的法向量
        fpfh.setSearchMethod(tree);
        fpfh.setRadiusSearch(this->get_parameter("fpfh_radius").as_double());        
        fpfh.compute(*features);   
    }

    // 粗匹配：FPFH+RANSAC
    Eigen::Matrix4f coarseRegistration(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud)
    {
        // 计算实时点云的FPFH特征
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_features(new pcl::PointCloud<pcl::FPFHSignature33>());
        pcl::search::KdTree<pcl::PointNormal>::Ptr cloud_tree(new pcl::search::KdTree<pcl::PointNormal>());
        computeFPFHFeatures(cloud, cloud_features, cloud_tree);

        // 特征匹配
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> ce;
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        ce.setInputSource(cloud_features);
        ce.setInputTarget(map_features_);
        ce.determineCorrespondences(*correspondences, 0.5);  // 特征距离阈值

        // RANSAC剔除错误匹配
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> ransac;  
        ransac.setInputSource(cloud);
        ransac.setInputTarget(filtered_map_);  
        ransac.setInputCorrespondences(correspondences);
        ransac.setInlierThreshold((float)ransac_threshold_);  // 几何一致性阈值
        ransac.getCorrespondences(*correspondences);

        return ransac.getBestTransformation();
    }

    // 精匹配：Point-to-Plane ICP
    Eigen::Matrix4f fineRegistration(
        const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
        const Eigen::Matrix4f& initial_guess) 
    {
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(filtered_map_);
        icp.setMaximumIterations(this->get_parameter("icp_iterations").as_int());
        icp.setTransformationEpsilon(1e-5);
        icp.setEuclideanFitnessEpsilon(1e-4);

        pcl::PointCloud<pcl::PointNormal> aligned_cloud;
        icp.align(aligned_cloud, initial_guess);  // 传入初始变换

        // 保存当前ICP匹配误差
        last_icp_fitness_ = icp.getFitnessScore();

        if (icp.hasConverged()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                              "ICP匹配成功，误差: %.4f", last_icp_fitness_);
            // 更新成功时间
            if (last_icp_fitness_ < icp_fitness_threshold_) {
                last_successful_time_ = this->get_clock()->now();
            }
            return icp.getFinalTransformation();
        } else {
            RCLCPP_WARN(this->get_logger(), "ICP匹配失败，使用初始猜测");
            return initial_guess;
        }
    }

    // 融合IMU与点云位姿（简单加权）
    Eigen::Matrix4f fusePoses(
        const Eigen::Matrix4f& lidar_pose,
        const Eigen::Matrix4f& imu_pose)
    {
        Eigen::Matrix4f fused = Eigen::Matrix4f::Identity();
        // 旋转融合：使用IMU的短期稳定性和雷达的长期校正
        static Eigen::Matrix3f last_rot = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f lidar_rot = lidar_pose.block<3,3>(0,0);
        Eigen::Matrix3f imu_rot = imu_pose.block<3,3>(0,0);
        
        // 互补滤波：雷达旋转慢更新，IMU旋转快更新
        fused.block<3,3>(0,0) = 0.9 * last_rot * imu_rot * last_rot.inverse() + 0.1 * lidar_rot;
        last_rot = fused.block<3,3>(0,0);  // 保存当前旋转用于下次计算
        
        // 位置融合：添加低通滤波减少跳变
        static Eigen::Vector3f last_pos = Eigen::Vector3f::Zero();
        Eigen::Vector3f current_pos = lidar_pose.block<3,1>(0,3);
        fused.block<3,1>(0,3) = 0.8 * last_pos + 0.2 * current_pos;  // 平滑位置
        last_pos = fused.block<3,1>(0,3);
        return fused;
    }

    // 发布轨迹和TF
    void publishTrajectory(const Eigen::Matrix4f& transform, const rclcpp::Time& stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = stamp;
        pose.header.frame_id = map_frame_;
        
        // 位置
        pose.pose.position.x = transform(0,3);
        pose.pose.position.y = transform(1,3);
        pose.pose.position.z = transform(2,3);
        
        // 姿态（旋转矩阵转四元数）
        Eigen::Quaternionf quat(transform.block<3,3>(0,0));
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();

        // 更新路径
        path_.header = pose.header;
        path_.poses.push_back(pose);
        if (path_.poses.size() > 2000) path_.poses.erase(path_.poses.begin());
        path_pub_->publish(path_);

        // 发布TF
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
    bool is_initialized_ = false;
    bool trigger_relocalization_ = false;
    double voxel_size_;
    double ransac_threshold_;
    double icp_fitness_threshold_;  // ICP匹配误差阈值
    double max_idle_time_;          // 最大空闲时间(秒)
    double last_icp_fitness_ = 0.0; // 上次ICP匹配误差
    rclcpp::Time last_successful_time_;  // 上次成功定位时间

    // 位姿存储
    Eigen::Matrix4f last_pose_;
    Eigen::Matrix4f imu_pose_;

    // 点云数据
    pcl::PointCloud<pcl::PointNormal>::Ptr filtered_map_ = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr map_features_ = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
    pcl::search::KdTree<pcl::PointNormal>::Ptr map_tree_ = std::make_shared<pcl::search::KdTree<pcl::PointNormal>>();
    
    // 订阅与发布
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr manual_reloc_param_sub_;

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