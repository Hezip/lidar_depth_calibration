/*
 * @Copyright: 2020, Authors. All rights reserved.
 * @Description: Match the input point cloud, and manually reconfig to avoid local minimums
 * @Author: v_hezhenpeng
 * @Date: 2020-8-18
 */

#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_depth_calibration/TutorialsConfig.h>

#include <string>
#include <algorithm>
#include <vector>

#include "common.hpp"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#define pi 3.1415926

PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
bool next_iteration = false;
bool reconfig = false;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();  // Transformation result

/*
 * @brief: Input point cloud is too big and need a filter
 */
void filter() {
  float voxel_size = 0.1;
  ros::param::get("voxel_size", voxel_size);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_icp, *cloud_icp, indices);
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, indices);
  pcl::VoxelGrid<pcl::PointXYZ> grid;
  grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  grid.setInputCloud(cloud_icp);
  grid.filter(*cloud_icp);
  grid.setInputCloud(cloud_in);
  grid.filter(*cloud_in);
}

/*
 * @brief
 */
bool loadPointCloud () {
  std::string input_pcd_path;
  if(!ros::param::get("input_pcd_path", input_pcd_path)) {
    ROS_ERROR("wrong input path");
    return false;
  }
  pcl::console::print_highlight ("Loading point clouds...\n");
  if(pcl::io::loadPCDFile<PointT>(input_pcd_path + "point_all.pcd", *cloud_in) < 0 ||
      pcl::io::loadPCDFile<PointT>(input_pcd_path + "image_all.pcd", *cloud_icp) < 0) {
    pcl::console::print_error("Error loading pcd file!\n");
    return false;
  }
  filter();
  return true;
}

/*
 * @brief:  PCL viewer keyboard callback :
 */
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent& event, void* nothing) {
  if(event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
}

/*
 * @brief: ROS Dynamic reconfig call back, transform point cloud by rqt_reconfigure
 */
void dynamicCallback(lidar_depth_calibration::TutorialsConfig &config, uint32_t level) {
  Eigen::Vector3d eulerAngle(config.yaw*pi/180,config.pitch*pi/180,config.roll*pi/180);
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()));
  Eigen::Matrix3d rotation_matrix;
  rotation_matrix=yawAngle*pitchAngle*rollAngle;
  Eigen::Vector3d translation(config.x, config.y, config.z);
  Eigen::Matrix4d transform;
  transform.block<3,3>(0, 0) = rotation_matrix;
  transform.block<3,1>(0, 3) = translation;
  transform(3, 3) = 1;
  pcl::transformPointCloud(*cloud_icp, *cloud_icp, transform);
  transformation_matrix *= transform;
  config.x = 0; config.y = 0; config.z = 0;
  config.roll = 0; config.pitch = 0; config.yaw = 0;
  reconfig = true;
}

int main (int argc, char* argv[]) {
  ros::init(argc, argv, "registered");
  dynamic_reconfigure::Server<lidar_depth_calibration::TutorialsConfig> server;
  dynamic_reconfigure::Server<lidar_depth_calibration::TutorialsConfig>::CallbackType f;
  f = boost::bind(&dynamicCallback, _1, _2);
  server.setCallback(f);

  if (!loadPointCloud()) return false;
  int iterations = 1;  // Default number of ICP iterations

//  pcl::IterativeClosestPoint<PointT, PointT> icp;
   pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations (iterations);
  icp.setMaxCorrespondenceDistance(0.1);
  icp.setInputSource (cloud_icp);
  icp.setInputTarget (cloud_in);
  icp.align (*cloud_icp);
  icp.setMaximumIterations (10);  // We set this variable to 10 for the next time we will call .align () function

  if (icp.hasConverged ()) {
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
  }
  else {
    PCL_ERROR ("\nICP has not converged.\n");
    return (-1);
  }

  // Visualization
  pcl::visualization::PCLVisualizer viewer ("ICP demo");
  float bckgr_gray_level = 0.3;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud_in, 255.0, 0.0, 255.0);
  viewer.addPointCloud (cloud_in, cloud_in_color_h, "cloud_in_v2");
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h (cloud_icp, 20, 180, 20);
  viewer.addPointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
  viewer.addText ("Pink: Original point cloud\nGreen: ICP aligned point cloud", 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "icp_info_2");
  viewer.addCoordinateSystem(1.0);
  std::stringstream ss;
  ss << iterations;
  std::string iterations_cnt = "ICP iterations = " + ss.str ();
  viewer.addText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
  viewer.setBackgroundColor (bckgr_gray_level, bckgr_gray_level, bckgr_gray_level);
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_icp_v2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud_in_v2");
  viewer.setCameraPosition(-3,0,0,0,0,1,0);
  viewer.setSize (1280, 1024);  // Visualiser window size
  viewer.registerKeyboardCallback (&keyboardEventOccurred, (void*) NULL);

  // Display the visualiser
  while (!viewer.wasStopped ()) {
    ros::spinOnce();
    viewer.spinOnce();
    // The user pressed "space" :
    if (next_iteration) {
      icp.align (*cloud_icp);

      if (icp.hasConverged ()) {
        iterations += 10;
        transformation_matrix *= icp.getFinalTransformation ().cast<double>();
        // print and update information
        {
          printf ("\nICP has converged, score is %+.0e\n", icp.getFitnessScore ());
          print4x4Matrix (transformation_matrix);
          ss.str ("");
          ss << iterations;
          std::string iterations_cnt = "ICP iterations = " + ss.str ();
          viewer.updateText (iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt");
          viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
        }
      }
      else {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
      }
    } else if (reconfig){
      reconfig = false;
      viewer.updatePointCloud (cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
      print4x4Matrix (transformation_matrix);
    }
    next_iteration = false;
  }
  return (0);
}
