import coor_trans
from pointcloud import PointCloud


pcd_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/制图pcap/map.pcd"
pcd_20m = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m.pcd"


file_20m = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m.txt"
file_40m = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/40m/40m.txt"
file_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/map/map.txt"
car_20m_lon = 125.1572392
car_20m_lat = 43.8327755
car_20m_heading = 199.82

car_40m_lon = 125.1573837
car_40m_lat = 43.8324837
car_40m_heading = 199.59

car_map_lon = 125.1569996
car_map_lat = 43.8332673
car_map_heading = 200.94

file_20m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m_gps.txt"
file_40m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/40m/40m_gps.txt"
file_map_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/map/map_gps.txt"

file_20m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m_in_map.txt"
file_40m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/40m/40m_in_map.txt"

f_20m = open(file_20m, "r+")
f_20m_lines = f_20m.readlines()
for i in range(len(f_20m_lines)):
    classification, occlusion, length, width, height, x, y, z, theta, angle, score = f_20m_lines[
        i].rstrip().split('\t')
    obj_lon, obj_lat, obj_heading = coor_trans.lidar_coor2gps(
        car_20m_lon, car_20m_lat, car_20m_heading, float(x), float(y),
        float(angle))
    f_20m_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_lon, obj_lat, z,
        theta, obj_heading, score)
f_20m_gps = open(file_20m_gps, 'w+')
f_20m_gps.writelines(f_20m_lines)

f_20m_gps = open(file_20m_gps, "r+")
f_20m_gps_lines = f_20m_gps.readlines()
for i in range(len(f_20m_gps_lines)):
    classification, occlusion, length, width, height, lon, lat, z, theta, heading, score = f_20m_gps_lines[
        i].rstrip().split('\t')
    obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor(
        car_map_lon, car_map_lat, car_map_heading, float(lon), float(lat),
        float(heading))
    f_20m_gps_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_x, obj_y, z,
        theta, obj_angle, score)
f_20m_in_map = open(file_20m_in_map, 'w+')
f_20m_in_map.writelines(f_20m_gps_lines)
f_20m_in_map.close()


pc = PointCloud(channel_num=4, filename=pcd_map)
pc.create_vis()
pred_boxes = pc.draw_3dboxes_from_txt(file_20m_in_map, [1, 1, 1])
pc.display_pc()
