/*
 * @Author: wentao zhang && zwt190315@163.com
 * @Date: 2023-04-24
 * @LastEditTime: 2023-07-22
 * @Description: 
 * @reference: 
 * 
 * Copyright (c) 2023 by wentao zhang, All Rights Reserved. 
 * @TODO:
 * 1. add local map publisher with realtime odometry height
 */
#include "Fiesta.h"

std::shared_ptr<fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, nav_msgs::Odometry::ConstPtr>> fiesta_map;
// std::shared_ptr<fiesta::Fiesta<sensor_msgs::PointCloud::ConstPtr, nav_msgs::Odometry::ConstPtr>> fiesta_map;

int main(int argc, char **argv) {
  ros::init(argc, argv, "FIESTA");
  ros::NodeHandle node("~");
  // fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::TransformStamped::ConstPtr> esdf_map(node);
  fiesta_map = std::make_shared<fiesta::Fiesta<sensor_msgs::PointCloud2::ConstPtr, nav_msgs::Odometry::ConstPtr>>(node);
  // fiesta_map = std::make_shared<fiesta::Fiesta<sensor_msgs::PointCloud::ConstPtr, nav_msgs::Odometry::ConstPtr>>(node);
  // esdf_map->esdf_map_;
  /* esdf访问示例
  */
  fiesta_map->esdf_map_->GetDistance(Eigen::Vector3d(0, 0, 0));
  fiesta_map->esdf_map_->GetOccupancy(Eigen::Vector3d(0, 0, 0));
  ros::spin();
  return 0;
}
