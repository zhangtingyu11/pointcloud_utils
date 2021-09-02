#include "pointcloud.h"

int main(int argc, char ** argv){
        if(argc != 3){
                std::cerr << "Must Enter 2 Filename!" << std::endl;
        }
        const char * binary_file = argv[2];
        const char * ascii_file = argv[1];
        PointCloud pc_instance;
        pc_instance.read_pcd_file(ascii_file);
        pc_instance.write_binary_pcd_file(binary_file);
        return 0;
}