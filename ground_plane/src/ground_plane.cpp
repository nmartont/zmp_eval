/*
 Author: Marton
 */

#include <iostream>
#include "ground_plane/ground_plane.h"

int main (int argc, char** argv)
{
  // We have an (x, y, z, intensity) point cloud, load it into an object
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("p_c.pcd", *cloud) == -1) //* load the file
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

  return (0);
}
