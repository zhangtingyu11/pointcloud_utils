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

void PointCloud::readKittiPclBinData(std::string &in_file, std::string& out_file)
{
    // load point cloud
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good()){
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

    int i;
    for (i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        points->push_back(point);
    }
    input.close();
//    g_cloud_pub.publish( points );

    std::cout << "Read KTTI point cloud with " << i << " points, writing to " << out_file << std::endl;
    pcl::PCDWriter writer;

    // Save DoN features
    writer.write< pcl::PointXYZI > (out_file, *points, false);
}