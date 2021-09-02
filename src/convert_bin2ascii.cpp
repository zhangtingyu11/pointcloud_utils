#include "pointcloud.h"

int main(int argc, char ** argv){
        if(argc != 2){
                std::cerr << "Must Enter 2 Filename!" << std::endl;
        }
        const char * binary_file = argv[1];
        const char * ascii_file = argv[2];
        PointCloud pc_instance;
        pc_instance.read_pcd_file(binary_file);
        pc_instance.write_ascii_pcd_file(ascii_file);
        return 0;
}