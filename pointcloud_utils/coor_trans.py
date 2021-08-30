import pyproj
import math


# IMU到lidar在车辆x方向的偏移量，值表示Lidar在车辆坐标系下的x坐标 - IMU在车辆坐标系下的x坐标
# 没有y偏移是因为目前y偏移为0
IMU_2_LIDAR_X_OFFSET = 0.9
geod = pyproj.Geod(ellps="WGS84")


def lidar_coor2gps(car_lon, car_lat, car_heading, obj_x, obj_y, obj_angle):
    """[将激光雷达坐标系下的障碍物转换到gps坐标]

    Args:
        car_lon ([type]): [主车的经度]
        car_lat ([type]): [主车的纬度]
        car_heading ([type]): [主车的航向角， 与正北方向夹角，逆时针为正]
        obj_x ([type]): [障碍物在lidar坐标系下的x坐标]
        obj_y ([type]): [障碍物在lidar坐标系下的y坐标]
        obj_angle ([type]): [障碍物和主车航向的夹角，顺时针为正]
    """
    # 计算障碍物距离车辆后轴中心点的距离
    distance = math.sqrt(
        math.pow(obj_x + IMU_2_LIDAR_X_OFFSET, 2) + math.pow(obj_y, 2))
    # 障碍物和车辆后轴中心点的连线和车头方向的夹角（0-360，逆时针为正）
    obj2car_angle = math.degrees(math.atan2(obj_y, obj_x+IMU_2_LIDAR_X_OFFSET))
    # 获取障碍物的航向角（与正北方向夹角，顺时针为正）
    azm = 360 - ((obj2car_angle + car_heading) % 360)
    obj_lon, obj_lat, _ = geod.fwd(car_lon, car_lat, azm, distance)
    obj_heading = ((car_heading - obj_angle) % 360)
    # 返回的航向角和正北方夹角，逆时针为正
    return obj_lon, obj_lat, obj_heading


def gps2lidar_coor(car_lon, car_lat, car_heading, obj_lon, obj_lat, obj_heading):
    """[将障碍物的经纬度转化为激光雷达坐标系的坐标]

    Args:
        car_lon ([type]): [主车经度]
        car_lat ([type]): [主车纬度]
        car_heading ([type]): [主车航向角， 与正北方向夹角，逆时针为正]
        obj_lon ([type]): [障碍物经度]
        obj_lat ([type]): [障碍物纬度]
        obj_heading ([type]): [障碍物航向角，和正北方夹角，逆时针为正]
    """
    forward_amz, _, distance = geod.inv(car_lon, car_lat, obj_lon, obj_lat)
    # 障碍物和主车的夹角，顺时针为正，0-360
    obj2car_angle = (forward_amz + car_heading) % 360
    obj_x = distance * math.cos(math.radians(obj2car_angle)) - IMU_2_LIDAR_X_OFFSET
    obj_y = distance * math.sin(math.radians(obj2car_angle))
    obj_angle = (car_heading - obj_heading) % 360
    return obj_x, -obj_y, obj_angle
