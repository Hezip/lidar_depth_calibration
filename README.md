# lidar_depth_calibration

激光雷达与深度相机zed半自动标定

## 文件说明

load_data.launch: 读取rosbag文件，将激光雷达点云和深度图像点云转换到世界坐标系下(根据提供的初始外参)

registered.launch: 匹配激光点云和深度图像点云，可以通过rqt_reconfigure手动设置点云变换来避免ICP陷入局部最小值


## 快速开始

## Part 1  

读取数据

1. load_data_config中设置 rosbag路径 内外参路径 输出路径
    - lidar topic 为 /livox/lidar_all
    - depth topic 为 /zed/zed_node/depth/depth_registered
    - 默认存储1帧深度图像点云 50帧激光雷达点云 

2. 设置/data目录下内参外参
    - 外参可以通过 rosrun tf tf_echo base_link livox_frame 给定初始值

3. roslaunch lidar_depth_calibration load_data.launch
    - 可在给定输出路径下看到 img_all.pcd point_all.pcd

## Part 2

点云半自动匹配

1. registered_config中设置 输入点云路径(即Part1.1中输出路径) voxel_fliter 大小
   
2. roslaunch lidar_depth_calibration registered.launch
   - 鼠标左键调整视角 中键调整位置 右键调整距离
   - 按下空格 进行10次GICP迭代 结果输出在控制台上

3. rosrun rqt_reconfigure rqt_reconfigure
   - 通过bar设置点云的一次transform 旋转中心为坐标原点
   - 每次设置完后bar会被清零

4. 匹配结果输出在控制台上



### 运行测试
1. 测试bag数据 暂不提供
2. 测试pcd数据 (https://drive.google.com/drive/folders/1aQjl0Pf8HW0tGF9ss3jZVvq6aX-KYVom?usp=sharing)
3. image_all.pcd 和 point_all.pcd 放在/data目录下
4. 进入Part2流程