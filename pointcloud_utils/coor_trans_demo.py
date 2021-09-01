import coor_trans
from pointcloud import PointCloud
import numpy as np
import open3d as o3d

pcd_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/map/map.pcd"
pcd_20m = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m.pcd"
pcd_road = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/map/map.pcd"


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

road_lon = 125.15707001863602
road_lat = 43.832977952695146
road_heading = 295.3


file_20m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m_gps.txt"
file_40m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/40m/40m_gps.txt"
file_map_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/map/map_gps.txt"

file_20m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/20m/20m_in_map.txt"
file_40m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/40m/40m_in_map.txt"
file_road_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/road/road_in_map.txt"

file_road = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/road/road_in_road.txt"
file_road_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/road/road_gps.txt"

file_fusion_20m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/fusion/fusion_20m_gps.txt"
file_fusion_20m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/fusion/fusion_20m_in_map.txt"
file_fusion_40m_gps = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/fusion/fusion_40m_gps.txt"
file_fusion_40m_in_map = "/home/zty/My_Project/pointcloud_utils/data/pcd_files/fusion/fusion_40m_in_map.txt"

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


f_40m = open(file_40m, "r+")
f_40m_lines = f_40m.readlines()
for i in range(len(f_40m_lines)):
    classification, occlusion, length, width, height, x, y, z, theta, angle, score = f_40m_lines[
        i].rstrip().split('\t')
    obj_lon, obj_lat, obj_heading = coor_trans.lidar_coor2gps(
        car_40m_lon, car_40m_lat, car_40m_heading, float(x), float(y),
        float(angle))
    f_40m_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_lon, obj_lat, z,
        theta, obj_heading, score)
f_40m_gps = open(file_40m_gps, 'w+')
f_40m_gps.writelines(f_40m_lines)
f_40m_gps.close()

f_40m_gps = open(file_40m_gps, "r+")
f_40m_gps_lines = f_40m_gps.readlines()
for i in range(len(f_40m_gps_lines)):
    classification, occlusion, length, width, height, lon, lat, z, theta, heading, score = f_40m_gps_lines[
        i].rstrip().split('\t')
    obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor(
        car_map_lon, car_map_lat, car_map_heading, float(lon), float(lat),
        float(heading))
    f_40m_gps_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_x, obj_y, z,
        theta, obj_angle, score)
f_40m_in_map = open(file_40m_in_map, 'w+')
f_40m_in_map.writelines(f_40m_gps_lines)
f_40m_in_map.close()


# f_road = open(file_road, "r+")
# f_road_lines = f_road.readlines()
# for i in range(len(f_road_lines)):
#     classification, occlusion, length, width, height, x, y, z, theta, angle, score = f_road_lines[
#         i].rstrip().split('\t')
#     obj_lon, obj_lat, obj_heading = coor_trans.lidar_coor2gps_road(
#         road_lon, road_lat, road_heading, float(x), float(y),
#         float(angle))
#     f_road_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
#         classification, occlusion, length, width, height, obj_lon, obj_lat, z,
#         theta, obj_heading, score)
# f_road_gps = open(file_road_gps, 'w+')
# f_road_gps.writelines(f_road_lines)
# f_road_gps.close()

f_road_gps = open(file_road_gps, "r+")
f_road_gps_lines = f_road_gps.readlines()
for i in range(len(f_road_gps_lines)):
    classification, occlusion, length, width, height, lon, lat, z, theta, heading, score = f_road_gps_lines[
        i].rstrip().split('\t')
    obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor_road(
        car_map_lon, car_map_lat, car_map_heading, float(lon), float(lat),
        float(heading))
    f_road_gps_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_x, obj_y, z,
        theta, obj_angle, score)
f_road_in_map = open(file_road_in_map, 'w+')
f_road_in_map.writelines(f_road_gps_lines)
f_road_in_map.close()


f_fusion_20m_gps = open(file_fusion_20m_gps, "r+")
f_fusion_20m_gps_lines = f_fusion_20m_gps.readlines()
for i in range(len(f_fusion_20m_gps_lines)):
    classification, occlusion, length, width, height, lon, lat, z, theta, heading, score = f_fusion_20m_gps_lines[
        i].rstrip().split('\t')
    obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor(
        car_map_lon, car_map_lat, car_map_heading, float(lon), float(lat),
        float(heading))
    f_fusion_20m_gps_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_x, obj_y, z,
        theta, obj_angle, score)
f_fusion_20m_in_map = open(file_fusion_20m_in_map, 'w+')
f_fusion_20m_in_map.writelines(f_fusion_20m_gps_lines)
f_fusion_20m_in_map.close()


f_fusion_40m_gps = open(file_fusion_40m_gps, "r+")
f_fusion_40m_gps_lines = f_fusion_40m_gps.readlines()
for i in range(len(f_fusion_40m_gps_lines)):
    classification, occlusion, length, width, height, lon, lat, z, theta, heading, score = f_fusion_40m_gps_lines[
        i].rstrip().split('\t')
    obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor(
        car_map_lon, car_map_lat, car_map_heading, float(lon), float(lat),
        float(heading))
    f_fusion_40m_gps_lines[i] = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}\n".format(
        classification, occlusion, length, width, height, obj_x, obj_y, z,
        theta, obj_angle, score)
f_fusion_40m_in_map = open(file_fusion_40m_in_map, 'w+')
f_fusion_40m_in_map.writelines(f_fusion_40m_gps_lines)
f_fusion_40m_in_map.close()

pc = PointCloud(channel_num=4, filename=pcd_map)
pc.create_vis()
# # 显示路侧数据
# obj_x, obj_y, obj_angle = coor_trans.gps2lidar_coor(car_map_lon, car_map_lat,
#                                                     car_map_heading,
#                                                     float(road_lon),
#                                                     float(road_lat),
#                                                     float(276))
# points_box = [[obj_x - 0.5, obj_y - 0.5, 1], [obj_x - 0.5, obj_y + 0.5, 1], [obj_x + 0.5, obj_y + 0.5, 1], [obj_x + 0.5, obj_y - 0.5, 1]]
# points_box = np.asarray(points_box)
# lines_box = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])
# color = [1, 0, 0]
# colors = np.array([color for j in range(len(lines_box))])
# line_set = o3d.geometry.LineSet()
# line_set.points = o3d.utility.Vector3dVector(points_box)
# line_set.lines = o3d.utility.Vector2iVector(lines_box)
# line_set.colors = o3d.utility.Vector3dVector(colors)
# pc.vis.add_geometry(line_set)
# pred_boxes = pc.draw_3dboxes_from_txt(file_map, [1, 1, 1])
road_boxrs = pc.draw_3dboxes_from_txt(file_fusion_40m_in_map, [1, 0, 0])


pc.display_pc()
