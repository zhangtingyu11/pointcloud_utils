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
        def __init__(self, center_x, center_y,width, height, angle = 0.0, type = None, score = 0, lidar_xy = True):
                """[初始化2D包围框]
                lidar_xy为True的坐标系：
                                        x
                                        ^
                                        |
                                        |
                                        |
                                        |
                          y<-------------
                lidar_xy为False的坐标系：
                        --------------->x
                        |
                        |
                        |
                        |
                        |
                        |
                        v
                        y  
                宽为y轴上的长度，高为x轴上的高度
                Args:
                    center_x ([type]): [包围框的中心点x坐标]
                    center_y ([type]): [包围框的中心点y坐标]
                    width ([type]): [包围框宽]
                    height ([type]): [包围框高]
                    angle (float, optional): [包围框顺时针旋转的角度]. Defaults to 0.
                    type ([type], optional): [包围框的类别]. Defaults to None.
                    score ([type], optional): [包围框的置信度]. Defaults to None.
                    lidar_xy ([bool], optional): [是否使用lidar_xy坐标系]. Defaults to True.
                """
                self.center_x = center_x
                self.center_y = center_y
                self.width = width
                self.height = height
                self.angle = angle
                self.type = type
                self.score = score
                self.lidar_xy = lidar_xy
        
        def get_corners(self)->np.array:
                """[获取此顺序下的角点坐标]
                0----------1
                |          |
                |          |
                |          |
                |          |
                3----------2

                Returns:
                    np.array: [4*2的角点坐标]
                """
                corners_2d = []
                corner_0 = [self.height/2, self.width/2]
                corners_2d.append(corner_0)
                corner_1 = [self.height/2, -self.width/2]
                corners_2d.append(corner_1)
                corner_2 = [-self.height/2, -self.width/2]
                corners_2d.append(corner_2)
                corner_3 = [-self.height/2, self.width/2]
                corners_2d.append(corner_3)
                corners_2d = np.asarray(corners_2d)
                if(self.lidar_xy):
                        rotate_matrix = self.get_2d_rotate_matrix(self.angle)
                        corners_2d = np.transpose(np.dot(rotate_matrix, np.transpose(corners_2d)))
                else:
                        #坐标系相反需要调整
                        rotate_matrix = self.get_2d_rotate_matrix(-self.angle)
                        corners_2d = np.transpose(np.dot(rotate_matrix, np.transpose(corners_2d))) 
                corners_2d[:,0] += self.center_x
                corners_2d[:,1] += self.center_y
                return corners_2d

        @staticmethod
        def get_2d_rotate_matrix(angle):
                """[获取2D逆时针旋转的角度]

                Args:
                    angle ([type]): [逆时针旋转的角度]

                Returns:
                    [type]: [2*2的旋转矩阵]
                """
                cos_theta = math.cos(angle)
                sin_theta = math.sin(angle)
                return np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])
                
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
                poly1 = Polygon(self.get_corners()).convex_hull
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
                

                
        @staticmethod
        def compute_2d_iou(bbox1, bbox2)->float:
                """计算两个2D包围框的iou

                Args:
                        bbox1 (BoundingBox_2d): [其中一个2D包围框]
                        bbox2 (BoundingBox_2d): [另一个2D包围框]

                Returns:
                        double: [返回iou的值]
                """
                poly1 = Polygon(bbox1.get_corners()).convex_hull
                poly2 = Polygon(bbox2.get_corners()).convex_hull
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
        def __init__(self, length, width, height, location_x, location_y, location_z, angle = 0, type = None, score = 0):
                """[初始化3D包围框类]
                x轴向前，y轴向左，z轴向上，长为x轴上的长度，宽为y轴上的长度，高度为z轴上的长度

                Args:
                    length ([type]): [障碍物的长]
                    width ([type]): [障碍物的宽]
                    height ([type]): [障碍物的高]
                    location_x ([type]): [障碍物中心点的x位置]
                    location_y ([type]): [障碍物中心点的y位置]
                    location_z ([type]): [障碍物中心点的z位置]
                    angle (int, optional): [障碍物相对于主车顺时针旋转的角度(单位为弧度)]. Defaults to 0.
                    type ([type], optional): [障碍物类别，0为车，1为行人，2为非机动车]. Defaults to None.
                    score ([type], optional): [障碍物的分类置信度]. Defaults to 0.
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
                self.camera_intrinsic = None
                self.camera_extrinsic2lidar = None

        def get_3d_corners(self)->np.array:
                """[得到3D角点，顺序如下图所示]
                1 -------- 0
		/|         /|
		2 -------- 3 .
		| |       | |
		. 5 -------- 4
		|/        |/
		6 -------- 7

                Returns:
                    np.array: [8*3的角点坐标]
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

        def load_camera_intrinsic(self, camera_intrinsic:np.array):
                """[存储相机的内参]

                Args:
                    camera_intrinsic (np.array): [相机的3*4的相机内参]
                """
                self.camera_intrinsic = camera_intrinsic

        def load_camera_extrinsic2lidar(self, camera_extrinsic2lidar:np.array):
                """[加载相机外参]

                Args:
                    camera_extrinsic2lidar (np.array): [雷达相对于相机的外参的3*4的外参矩阵]
                """
                self.camera_extrinsic2lidar = camera_extrinsic2lidar

        def get_projected_2d_points(self)->np.array:
                """[获取3D点投影到图像上的2D点，需要提前调用load_camera_intrinsic和load_camera_extrinsic2lidar加载相机内外参]

                Returns:
                    np.array: [2D像素点的角点坐标]
                """
                if(self.camera_intrinsic == None):
                        assert("相机内参未加载，调用BoundingBox_3d.load_camera_intrinsic()加载")
                if(self.camera_extrinsic2lidar == None):
                        assert("相机外参未加载，调用BoundingBox_3d.load_camera_extrinsic2lidar()加载")
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

        @staticmethod
        def get_rotation_matrix_along_z(angle)->np.array:
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
                return BoundingBox_2d(center_x = (min_x+max_x)/2, center_y = (min_y+max_y)/2,
                        width = max_x-min_x, height = max_y-min_y)
        
        def get_bev_2d_box(self)->BoundingBox_2d:
                """[获得3D包围框投影到BEV视角的2D包围框]

                Returns:
                    BoundingBox_2d: [返回3D包围框投影到BEV视角的2D包围框]
                """
                return BoundingBox_2d(center_x = self.location_x, center_y=self.location_y,
                        width = self.width, height = self.length, angle = self.angle)

        def get_bev_2d_iou(self, d3bbox2)->float:
                """[计算BEV视角下的2Diou]

                Args:
                    d3bbox2 ([BoundingBox_3d]): [另一个3D包围框]

                Returns:
                    float: [iou的值]
                """
                return BoundingBox_2d.compute_2d_iou(self.get_bev_2d_box(), d3bbox2.get_bev_2d_box())


