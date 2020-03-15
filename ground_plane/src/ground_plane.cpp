/*
 Author: Marton
 */

#include <iostream>
#include "ground_plane/ground_plane.h"

bool is_cloud_ground_plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){
  /**
   * Decides if the given point cloud is a ground plane or not.
   *
   * @param cloud: The input point cloud
   * @return: true if the given cloud is a ground plane, false otherwise
   */

  // First, outlier removal
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (30);
  sor.setStddevMulThresh (0.1);
  sor.filter (*cloud_filtered);

#ifdef VISUALIZE
  // Display cloud
  pcl::visualization::CloudViewer viewer ("Filtered point cloud");
  viewer.showCloud (cloud_filtered);
  while (!viewer.wasStopped ()){}
#endif

  // Clustering
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
  ec.setClusterTolerance (0.5);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "Cluster " << j << ": " << cloud_cluster->points.size () << " data points." << std::endl;

#ifdef VISUALIZE
    // Display cloud
    pcl::visualization::CloudViewer viewer ("Cluster");
    viewer.showCloud (cloud_cluster);
    while (!viewer.wasStopped ()){}
#endif

    j++;
  }

  return true;  // TODO
}

int main (int argc, char** argv)
{
  // We have an (x, y, z, intensity) point cloud, load it into an object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("p_c.pcd", *cloud) == -1) // load the file
  {
    PCL_ERROR ("Couldn't read file\n");
    return (-1);
  }
  std::cout << "Loaded point cloud..\n";

#ifdef PRINT_DEBUG
  // Print out cloud data to verify that loading was successful
  for (std::size_t i = 0; i < cloud->points.size (); ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z
              << " "    << cloud->points[i].intensity << std::endl;
#endif

#ifdef VISUALIZE
  // Display cloud
  pcl::visualization::CloudViewer viewer ("Original point cloud");
  viewer.showCloud (cloud);
  while (!viewer.wasStopped ()){}
#endif

  // Call the ground plane detection function
  bool is_ground_plane;
  is_ground_plane = is_cloud_ground_plane(cloud);

  if(is_ground_plane) cout << "Ground plane: YES\n";
  else                cout << "Ground plane: NO\n";

  return (0);
}
