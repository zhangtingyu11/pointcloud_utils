#include "pointcloud.h"

PointCloud::PointCloud(){
        cloud.reset(new PointCloudType);
}

PointCloud::~PointCloud(){
}

void PointCloud::read_pcd_file(const char * filename){
        if (pcl::io::loadPCDFile(filename, *cloud) == -1)
        {
                PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
                return;
        }
}

void PointCloud::write_ascii_pcd_file(const char * filename){
        pcl::io::savePCDFileASCII (filename, *cloud);
}

void PointCloud::write_binary_pcd_file(const char * filename){
        pcl::io::savePCDFileBinary(filename, *cloud);
}