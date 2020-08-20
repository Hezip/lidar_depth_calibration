/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: Load data from rosbag
 *               Transform to the world frame through the given initial parameters
 * @Author: v_hezhenpeng
 * @Date: 2020-8-18
 */


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <iostream>
#include <string>
#include <vector>

#include "common.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std::vector<PointCloud> lidar_datas;
std::vector<PointCloud> image_cloud_datas;
int num_lidar_points;  // the maximum accept lidar frames
std::string input_bag_path, output_path, intrinsic_path, extrinsic_path;
Eigen::Matrix4d intrinsic, extrinsic_camera, extrinsic_lidar;

/*
 * @brief: Load ROS parameters and calculate the extrinsic
 */
void getParameters() {
  std::cout << "Get the parameters from the launch file" << std::endl;
  if (!ros::param::get("input_bag_path", input_bag_path)) {
    exit(1);
  }
  if (!ros::param::get("output_path", output_path)) {
    exit(1);
  }
  if (!ros::param::get("num_lidar_points", num_lidar_points)) {
    exit(1);
  }
  if (!ros::param::get("intrinsic_path", intrinsic_path)) {
    exit(1);
  }
  if (!ros::param::get("extrinsic_path", extrinsic_path)) {
    exit(1);
  }
  if (!loadMatrix(intrinsic_path + "/camera_intrinsic.txt", intrinsic)
      || !loadMatrix(extrinsic_path + "/lidar_extrinsic.txt", extrinsic_lidar)
      || !loadMatrix(extrinsic_path + "/camera_extrinsic.txt", extrinsic_camera)) {
    exit(1);
  }
  Eigen::Matrix4d camera_world;
  camera_world.setZero();
  camera_world(0, 2) = 1;
  camera_world(1, 0) = -1;
  camera_world(2, 1) = -1;
  camera_world(3, 3) = 1;
  extrinsic_camera *= camera_world;
}

/*
 * @brief: Convert depth image to point cloud
 *         Transform point cloud to world frame
 */
void convertDepth2Cloud(const sensor_msgs::ImageConstPtr& depth_msg, PointCloud::Ptr &output) {
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width  = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;
  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

  const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(float);

  float fx = intrinsic(0, 0);
  float fy = intrinsic(1, 1);
  float cx = intrinsic(0, 2);
  float cy = intrinsic(1, 2);

  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step) {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z) {
      float depth = depth_row[u];
      if (depth == 0) continue;
      *iter_x = (1.0*u - cx) * depth / fx;
      *iter_y =  (1.0*v - cy) * depth / fy;
      *iter_z = depth;
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg, *cloud);
  pcl::transformPointCloud(*cloud, *cloud, extrinsic_camera);
  *output = *cloud;
}

/*
 * @brief: Load lidar and depth camera point cloud
 */
void loadPointcloudFromROSBag(const std::string& bag_path) {
  ROS_INFO("Start to load the rosbag %s", bag_path.c_str());
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return;
  }

  std::vector<std::string> topics;
  topics.push_back(std::string("/livox/lidar_all"));  // message title
  topics.push_back(std::string("/zed/zed_node/depth/depth_registered"));  // message title
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view) {
    if (lidar_datas.size() < num_lidar_points) {
      sensor_msgs::PointCloud2ConstPtr lidar_m = m.instantiate<sensor_msgs::PointCloud2>();
      if (lidar_m == NULL) continue;
      PointCloud cloud;
      pcl::fromROSMsg(*lidar_m, cloud);
      pcl::transformPointCloud(cloud, cloud, extrinsic_lidar);
      lidar_datas.push_back(cloud);
    }
    if (image_cloud_datas.size() < 1) {
      sensor_msgs::ImageConstPtr depth_m = m.instantiate<sensor_msgs::Image>();
      if (depth_m == NULL) continue;
      PointCloud::Ptr cloud(new PointCloud);
      convertDepth2Cloud(depth_m, cloud);
      image_cloud_datas.push_back(*cloud);
    }
    if (image_cloud_datas.size() >= 1 && lidar_datas.size() >= num_lidar_points) {
      ROS_INFO("Finished load data");
      break;
    }
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "load_data");
  getParameters();
  loadPointcloudFromROSBag(input_bag_path);
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_all(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr image_all(new pcl::PointCloud<pcl::PointXYZ>());
  for (int i=0; i < lidar_datas.size(); i++) {
    *point_all += lidar_datas[i];
  }
  for (int i=0; i < image_cloud_datas.size(); i++) {
    *image_all += image_cloud_datas[i];
  }
  pcl::io::savePCDFileBinary(output_path + "point_all.pcd", *point_all);
  pcl::io::savePCDFileBinary(output_path + "image_all.pcd", *image_all);
  ROS_INFO("Save cloud");
  return 0;
}
