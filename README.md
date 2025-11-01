# 导航组大作业相关说明

## 环境配置
Ubuntu 22.04 + ROS2 humble

请确保已经安装了Eigen、PCL等必须的库
```bash
mkdir src
cd src
```
- Livox-SDK2
```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
cd build && cmake ..
make
sudo make install
```

- livox_ros_driver2
```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
./build.sh humble
```

- FAST_LIO
```bash
git clone https://github.com/Ericsii/FAST_LIO.git --recursive
cd ..
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

```bash
colcon build --packages-select map_publisher relocalization
```

## 使用说明
- 建图
  - bash1:
    ```bash
    ros2 launch fast_lio mapping.launch.py
    ```
  - bash2:
    ```bash
    ros2 run map_publisher pcd_saver_node
    ```
  - bash3:
    ```bash
    ros2 bag play bag/easy
    ```
  - bash4:(bag播放结束后)
    ```bash
    ros2 service call /save_pcd std_srvs/srv/Empty "{}"
    ```
- 实时定位
  - bash1:
    ```bash
    ros2 run map_publisher map_publisher_node
    ```
  - bash2:
    ```bash
    ros2 run relocalization relocalization_node
    ```
  - 打开rviz2，选择map坐标系，订阅/static_pointcloud_map，添加Path并订阅/path
  - bash3:(选择normal/hard/lunatic测试包)
    ```bash
    ros2 bag play bag/normal
    ```
## 任务完成思路
- 成功部署FAST_LIO及所需的livox_ros_driver2,Livox-SDK2，即完成建图任务，没找到自带的保存地图的方式，写了pcd_saver_node用于地图保存，订阅/cloud_registered即可
- map_publihser_node读取本地保存的pcd地图并发布两个话题/static_pointcloud_map（用于RViz显示）和/map_for_navigation（用于重定位）
- relocalization_node订阅/map_for_navigation，/livox/lidar/pointcloud，/livox/imu，发布实时定位/path
## 算法流程及实现细节
- 重定位触发条件
  - 手动触发
  - 检测到剧烈运动（加速度超过阈值）
  - ICP匹配误差过大
  - 长时间未定位（距离上次成功定位时间超过阈值）
- 重定位流程
  - 粗匹配
    - 预处理：体素滤波+统计滤波
    - 特征提取：计算FPFH特征
    - 匹配提纯：根据距离阈值筛选初始匹配对，RANSAC剔除错误匹配，得到初始位姿
  - 精匹配
    - ICP
  - 位姿融合策略
    - 旋转：IMU占90%权重，雷达旋转占10%权重
    - 位置：上一时刻占80%权重，当前位置占20%权重
- 其他说明
  - 重定位算法部分很多不会，都是copilot在发力。
    