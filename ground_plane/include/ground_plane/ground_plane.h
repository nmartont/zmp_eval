/*
 Author: Marton
 */

#ifndef GROUND_PLANE_H_
#define GROUND_PLANE_H_

// ----------- Includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

// ----------- Function definitions
bool is_cloud_ground_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

// ----------- User defines
//#define PRINT_DEBUG
#define VISUALIZE

#endif  // GROUND_PLANE_H_
