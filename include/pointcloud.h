#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloudType;

class PointCloud{
        public:
                PointCloud();
                ~PointCloud();
                void read_pcd_file(const char * filename);
                void write_ascii_pcd_file(const char * filename);
                void write_binary_pcd_file(const char * filename);

        private:
                PointCloudType::Ptr cloud;

};


// int main (int argc, char** argv)
// {
//   pcl::PointCloud<PointCloudType>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

//   if (pcl::io::loadPCDFile<pcl::PointXYZ> ("test_pcd.pcd", *cloud) == -1) //* load the file
//   {
//     PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
//     return (-1);
//   }
//   std::cout << "Loaded "
//             << cloud->width * cloud->height
//             << " data points from test_pcd.pcd with the following fields: "
//             << std::endl;
//   for (const auto& point: *cloud)
//     std::cout << "    " << point.x
//               << " "    << point.y
//               << " "    << point.z << std::endl;

//   return (0);