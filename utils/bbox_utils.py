#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Description:       :
@Date     :2021/05/08 13:34:17
@Author      :zhangtingyu
@version      :1.0
'''
import numpy as np
import shapely
from shapely.geometry import Polygon
from shapely import affinity
from typing import List
import math

class BoundingBox_2d:
        def __init__(self, bbox_array:np.array, center_flag = False, angle = 0):
                """[summary]

                Args:
                    bbox_array (np.array): [用来表示包围框的数组]
                    center_flag (bool, optional): [是否使用2D框的中心表示法]. Defaults to False.
                        如果为True：代表使用中心表示法，bbox_array为[center_x, center_y, width, height, score, class]
                        如果为False：代表使用角点表示法，bbox_array为[left_upper_x, left_upper_y, right_bottom_x, right_bottom_y, score, class]
                    angle(double, optional):[绕着几何中心逆时针旋转的角度]. Defaults to 0.角度值，而非弧度
                """
                if(center_flag):
                        #包围框的左上角角点x坐标：中心点x-包围框宽度/2
                        self.left_upper_x = bbox_array[0] - bbox_array[2]/2
                        #包围框的左上角角点y坐标：中心点y-包围框高度/2
                        self.left_upper_y = bbox_array[1] - bbox_array[3]/2
                        #包围框的右下角角点x坐标：中心点x+包围框宽度/2
                        self.right_bottom_x = bbox_array[0] + bbox_array[2]/2
                        #包围框的右下角角点y坐标：中心点y+包围框高度/2
                        self.right_bottom_y = bbox_array[1] + bbox_array[3]/2
                else:
                        #包围框的左上角角点x坐标
                        self.left_upper_x = bbox_array[0]
                        #包围框的左上角角点y坐标
                        self.left_upper_y = bbox_array[1]
                        #包围框的右下角角点x坐标
                        self.right_bottom_x = bbox_array[2]
                        #包围框的右下角角点y坐标
                        self.right_bottom_y = bbox_array[3]
                        #包围框的左上角角点坐标
                #包围框的左上角角点坐标
                self.left_upper = np.array([self.left_upper_x, self.left_upper_y])
                #包围框的右下角角点坐标
                self.right_bottom = np.array([self.right_bottom_x, self.right_bottom_y])
                #包围框的左下角角点坐标
                self.left_bottom = np.array([self.left_upper_x, self.right_bottom_y])
                #包围框的右上角角点坐标
                self.right_upper = np.array([self.right_bottom_x, self.left_upper_y])

                #所有角点汇总成一个numpy数组(4x2)，需要逆时针选点
                self.points = np.vstack((self.left_upper, self.left_bottom, self.right_bottom, self.right_upper))
                if(angle != 0):
                        #绕着几何中心旋转angle角度，生成的是Polygon对象
                        rotated_points_poly = affinity.rotate(Polygon([(self.left_upper_x,self.left_upper_y), (self.right_bottom_x, self.left_upper_y), 
                                (self.right_bottom_x, self.right_bottom_y),(self.left_upper_x,self.right_bottom_y)]),angle)
                        #将Polygon转化为numpy数组，5x2，最后一个坐标点重复
                        rotated_points = np.asarray(rotated_points_poly.exterior.coords)
                        #重新赋值
                        self.points = rotated_points[0:4][:]
                        self.left_upper_x = rotated_points[0][0]
                        self.left_upper_y = rotated_points[0][1]
                        self.right_bottom_x = rotated_points[2][0]
                        self.right_bottom_y = rotated_points[2][1]
                        self.left_upper = rotated_points[0]
                        self.left_bottom = rotated_points[1]
                        self.right_bottom = rotated_points[2]
                        self.right_upper = rotated_points[3]

                #置信度(0~1)
                self.score = bbox_array[4]
                #类别(kitti中1是car，2是person，3是cyclelist)
                #(coco中person是0，bycycle是1，car是2，motorbike是3，bus是5，truck是7)
                #统一为car是1，person是2，cyclelist是3，bycycle是4，motorbike是5，bus是6，truck是7
                #对需要检测的目标类别做映射，其他的类别设置为-1
                if(bbox_array[5] == 2):
                        self.classification = 1
                elif(bbox_array[5] == 0):
                        self.classification = 2
                elif(bbox_array[5] == 1):
                        self.classification = 4
                elif(bbox_array[5] == 3):
                        self.classification = 5
                elif(bbox_array[5] == 5):
                        self.classification = 6
                else:
                        self.classification = -1              
        def bbox_in_rectangle(self, min_x, max_x, min_y, max_y)->bool:
                """[判断2D包围框在不在矩形范围内]

                Args:
                    min_x ([float]): [矩形的最小的x坐标]
                    max_x ([float]): [矩形的最大的x坐标]
                    min_y ([float]): [矩形的最小y坐标]
                    max_y ([float]): [矩形的最大y坐标]

                Returns:
                    bool: [如果在矩形内返回True，如果不在返回False]
                """
                poly1 = Polygon(self.points).convex_hull
                poly2 = Polygon([(min_x, min_y),(min_x, max_y),(max_x,max_y),(max_x,min_y)]).convex_hull
                if not poly1.intersects(poly2):
                        return False
                else:
                        try:
                                #交集面积
                                inter_area = poly1.intersection(poly2).area
                                if(inter_area == poly1.area):
                                        return True
                        except shapely.geos.TopologicalError:
                                print('shapely.geos.TopologicalError occured')
                                return False
                

                

def compute_2d_iou(bbox1:BoundingBox_2d, bbox2:BoundingBox_2d)->float:
        """计算两个2D包围框的iou

        Args:
                bbox1 (BoundingBox_2d): [其中一个2D包围框]
                bbox2 (BoundingBox_2d): [另一个2D包围框]

        Returns:
                double: [返回iou的值]
        """
        poly1 = Polygon(bbox1.points).convex_hull
        poly2 = Polygon(bbox2.points).convex_hull
        #如果没有交集，iou为0
        if not poly1.intersects(poly2):
                iou = 0
        else:
                try:
                        #交集面积
                        inter_area = poly1.intersection(poly2).area
                        #并集面积 = bbox1的面积+bbox2的面积-交集面积
                        union_area = poly1.area + poly2.area - inter_area
                        iou = inter_area/union_area
                except shapely.geos.TopologicalError:
                        print('shapely.geos.TopologicalError occured, iou set to 0')
                        iou = 0
        return iou

class BoundingBox_3d:
        def __init__(self, length, width, height, location_x, location_y, location_z, angle = 0, type = None, score = None):
                """[初始化3D包围框类]

                Args:
                    length ([type]): [障碍物的长]
                    width ([type]): [障碍物的宽]
                    height ([type]): [障碍物的高]
                    location_x ([type]): [障碍物中心点的x位置]
                    location_y ([type]): [障碍物中心点的y位置]
                    location_z ([type]): [障碍物中心点的z位置]
                    angle (int, optional): [障碍物相对于主车顺时针旋转的角度(单位为弧度)]. Defaults to 0.
                    type ([type], optional): [障碍物类别，0为车，1为行人，2为非机动车]. Defaults to None.
                    score ([type], optional): [障碍物的分类置信度]. Defaults to None.
                """
                self.length = length
                self.width = width
                self.height = height
                self.location_x = location_x
                self.location_y = location_y
                self.location_z = location_z
                self.angle = angle
                self.type = type
                self.score = score

        def get_3d_corners(self):
                """[得到3D角点，顺序如下图所示]
                1 -------- 0
		/|         /|
		2 -------- 3 .
		| |       | |
		. 5 -------- 4
		|/        |/
		6 -------- 7
                """
                corners_list = []
                corners_0 = [self.length/2, - self.width/2, self.height/2]
                corners_list.append(corners_0)
                corners_1 = [self.length/2, self.width/2, self.height/2]
                corners_list.append(corners_1)
                corners_2 = [-self.length/2, self.width/2, self.height/2]
                corners_list.append(corners_2)
                corners_3 = [-self.length/2, -self.width/2, self.height/2]
                corners_list.append(corners_3)
                corners_4 = [self.length/2, -self.width/2, -self.height/2]
                corners_list.append(corners_4)
                corners_5 = [self.length/2, self.width/2, -self.height/2]
                corners_list.append(corners_5)
                corners_6 = [-self.length/2, self.width/2, -self.height/2]
                corners_list.append(corners_6)
                corners_7 = [-self.length/2, -self.width/2, -self.height/2]
                corners_list.append(corners_7)
                corners_array = np.asarray(corners_list, dtype=np.float32)
                rotate_matrix = self.get_rotation_matrix_along_z(self.angle)
                corners_array = np.dot(rotate_matrix, np.transpose(corners_array))
                corners_array = np.transpose(corners_array)
                corners_array[:,0] += self.location_x
                corners_array[:,1] += self.location_y
                corners_array[:,2] += self.location_z

                return corners_array

        def load_camera_intrinsic(self, camera_intrinsic):
                self.camera_intrinsic = camera_intrinsic

        def load_camera_extrinsic2lidar(self, camera_extrinsic2lidar):
                self.camera_extrinsic2lidar = camera_extrinsic2lidar

        def get_projected_2d_points(self)->np.array:
                """[获取3D点投影到图像上的2D点，需要提前调用load_camera_intrinsic和load_camera_extrinsic2lidar加载相机内外参]

                Returns:
                    np.array: [2D像素点的角点坐标]
                """
                corners_3d = np.transpose(self.get_3d_corners())

                #将激光雷达坐标系下的3D点转化为齐次坐标([3x8]->[4x8])
                lidar_corners_3d = np.vstack((corners_3d, np.ones((1, 8))))
                #将激光雷达坐标系下的3D点转到相机坐标系下的3D点([3x4]*[4x8] = [3x8])
                camera_corners_3d= np.dot(self.camera_extrinsic2lidar, lidar_corners_3d)
                #将相机坐标系下的3D坐标转化为齐次坐标([3x8]->[4x8])
                camera_corners_3d_hom = np.vstack((camera_corners_3d, np.ones((1,8))))
                #将相机坐标系下的3D点转化到图像坐标系下的2D点([3x4]*[4x8] = [3x8])
                corners2d = np.dot(self.camera_intrinsic, camera_corners_3d_hom)
                #[3x8]的矩阵前两行的每个元素都除第三行才是像素点坐标
                corners2d[0,:]=corners2d[0,:]/corners2d[2,:]
                corners2d[1,:] = corners2d[1,:]/corners2d[2,:]
                #只取前两行([2x8])
                corners2d = np.transpose(corners2d[0:2,:])
                return corners2d

        def get_rotation_matrix_along_z(self, angle)->np.array:
                """[获取绕着z轴旋转的旋转矩阵，z轴为指向天的轴]
                Args:
                    angle ([type]): [逆时针旋转的角度]
                Returns:
                    np.array: [返回3x3的旋转矩阵]
                """

                cos_theta = np.cos(angle)
                sin_theta = np.sin(angle)
                return np.array([[cos_theta, -sin_theta, 0], [sin_theta, cos_theta, 0], [0, 0, 1]])

        def get_projected_2d_box(self)->np.array:
                """[获取最小的xy坐标和最大的xy坐标]

                Returns:
                    np.array: [[2x2]的矩阵，以行划分，第0行是左上角的点，第1行是右下角的点]
                """
                projected_2d_points = self.get_projected_2d_points()
                min_x = min(projected_2d_points[:][0])
                min_y = min(projected_2d_points[:][1])
                max_x = max(projected_2d_points[:][0])
                max_y = max(projected_2d_points[:][1])
                return BoundingBox_2d(np.array([min_x,min_y, max_x,max_y]))
        
        def get_bev_2d_box(self)->np.array:
                """[获得3D包围框投影到BEV视角的2D包围框]

                Returns:
                    np.array: [返回3D包围框投影到BEV视角的2D包围框]
                """
                return BoundingBox_2d(np.array([self.location_x,self.location_y,self.length,self.width]), center_flag=True, angle = self.angle)

        def get_bev_2d_iou(self, d3bbox2)->float:
                """[计算BEV视角下的2Diou]

                Args:
                    d3bbox2 ([BoundingBox_3d]): [另一个3D包围框]

                Returns:
                    float: [iou的值]
                """
                return compute_2d_iou(self.get_bev_2d_box(), d3bbox2.get_bev_2d_box())
        
        def transform_3d_points_to_open3d(self, points:np.array):
                """[将点云从激光雷达坐标系转换到open3d坐标系，只需要将点逆时针旋转90度]

                Args:
                    points (np.array): [N*3的点矩阵]
                """
                points = points[:,[1,0,2]]
                points[:, 0] = -points[:, 0]
                return points


