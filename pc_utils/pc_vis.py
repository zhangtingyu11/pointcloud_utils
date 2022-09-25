from xml.sax.handler import property_dom_node
import open3d.ml.torch as ml3d
import numpy as np
import os
import sys
sys.path.append("..")
import pclpy
from pc_utils.bbox import BoundingBox3D_O3D
from typing import List
import torch
import random
import string
import copy


class PCVisualizer():
    def __init__(self, 
            points = None,
            points_name = None, 
            pc_file = None,
            pc_dir = None, 
            points_dim = 4,
            pc_type=None) -> None:
        """创建一个点云可视化

        Args:
            points (np.ndarray, optional): 点云. Defaults to None.
            points_name (str, optional): 点云名称. Defaults to None.
            pc_file (_type_, optional): 需要读取的点云萎蔫. Defaults to None.
            pc_dir (_type_, optional): _description_. Defaults to None.
            points_dim (int, optional): _description_. Defaults to 4.
            pc_type (_type_, optional): _description_. Defaults to None.
        """
        self.points = {}
        if(points): #如果输入点那么点的优先级高于文件
            assert(isinstance(points_name, str)),"points_name must be str"
            self.addNewPointCloud(points, points_name)
        else:
            self.points = {}
            if(pc_file):
                pc = self.get_single_point_cloud_from_file(pc_file, pc_type, points_dim)
                self.addNewPointCloud(pc, points_name)
            elif(pc_dir):
                #TODO 读取一个文件夹内的点云文件
                self.pc_dir = pc_dir    
    
    @staticmethod
    def get_random_string():
        """获取随机字符串

        Returns:
            str: 随机字符串
        """
        return ''.join(random.sample(string.ascii_letters + string.digits, 8))

    def get_random_points_name(self):
        """获取随机的点云名称

        Returns:
            str: 点云名称
        """
        random_string = self.get_random_string()
        while(random_string in self.points):
            random_string = self.get_random_string()
        return random_string

    def add_single_point_cloud_from_points(self, points, points_name = None):
        """从点中添加一个点云

        Args:
            points (torch.Tensor or np.ndarray): 点云
            points_name (str, optional): 点云名称，不给的话就随机初始化一个. Defaults to None.
        """
        if(isinstance(points, torch.Tensor)):
            points = points.contiguous().cpu().numpy()
        assert(isinstance(points, np.ndarray)),"points should be tensor or np.ndarray,but got {pc_type}".format(pc_type=type(points))
        self.addNewPointCloud(points, points_name)
        
    def get_single_point_cloud_from_file(self, pc_file, pc_type, points_dim=4)->np.ndarray:
        """通过文件获取单个点云

        Args:
            pc_file (_type_): 存储点云的文件名
            pc_type:如果是pcd文件需要指定点云类型
            points_dim (int, optional): 点云的坐标维度+点云的特征维度. Defaults to 4.

        Raises:
            ValueError: 读取出来的点云的shape要么是一维的,要么是二维的

        Returns:
            np.ndarray: [N,坐标维度+特征维度]
        """
        assert(os.path.exists(pc_file)),\
            "No file named:{filename}"\
                .format(filename=os.path.abspath(pc_file))
        if(pc_file.endswith("bin")):  #如果输入是一个bin文件
            point_clouds = np.fromfile(pc_file, dtype=np.float32)
            if(len(point_clouds.shape)==1): #如果shape长度为1，那这个数量一定能整除点云的points_dim
                assert(point_clouds.shape[0]%points_dim==0), \
                    "points_shape must be divisible by points_dim,but points_shape:{ps}, points_dim:{pd}"\
                        .format(ps=point_clouds.shape[0], pd=points_dim)
            elif(len(point_clouds.shape)==2):    #如果shape长度是2，那么points_dim就要赋值为shape的第1维
                points_dim = point_clouds.shape[1]
            else:
                raise ValueError("Cannot process point cloud with shape dim not in [1,2]")
            point_clouds = point_clouds.reshape(-1, points_dim)   #reshape points
        elif(pc_file.endswith("pcd")):
            pcd_reader = pclpy.pcl.io.PCDReader()
            assert(pc_type is not None),"You must assign pc_type if you want to read pcd file."+\
                "supported_type:\n"+"1.PointXYZI"
            if(pc_type=="PointXYZI"):
                pc = pclpy.pcl.PointCloud.PointXYZI()
                pcd_reader.read(pc_file, pc)
                xyz = pc.xyz
                intensity = pc.intensity.reshape(-1, 1)
                point_clouds = np.concatenate([xyz, intensity], axis=1)
        return point_clouds

    def add_single_point_cloud_from_file(self, pc_file, pc_type=None, points_dim=4, points_name=None):
        """从点云文件中向点云可视化页面添加一个点云

        Args:
            pc_file (_type_): 点云文件的路径
            pc_type (_type_, optional): 如果是pcd文件,需要指定pcd的类型. Defaults to None.
            points_dim (int, optional): 点云的坐标维度+点云的特征维度. Defaults to 4.
        """
        point_cloud = self.get_single_point_cloud_from_file(pc_file, pc_type, points_dim)
        self.addNewPointCloud(point_cloud, points_name)

    def addNewPointCloud(self, point_cloud, points_name=None):
        if(points_name is None):
            points_name = self.get_random_points_name()
        assert(isinstance(points_name, str)),"points_name must be str, but got type:{pn_type}"\
            .format(pn_type=type(points_name))
        if(points_name in self.points):
            print("You have covered the points with name:"+points_name)
        else:
            self.points[points_name] = {}
        self.points[points_name]["points"] = point_cloud
        self.current_point_cloud_name = points_name

    @property
    def current_point_cloud_name(self):
        """获取当前处理的点云的名称

        Returns:
            _type_: _description_
        """
        return self._current_point_cloud_name

    @current_point_cloud_name.setter
    def current_point_cloud_name(self, value):
        """设置当前处理的点云的名称

        Args:
            value (str): 点云的名称
        """
        self._current_point_cloud_name = value


    def split_point_cloud(self):
        """
            将点云划分成坐标和特征
        """
        for pc_name, pc_dict in self.points.items():
            point_cloud = pc_dict["points"]
            self.points[pc_name]["coordinate"] = (point_cloud[:,0:3])
            self.points[pc_name]["feature"] = (point_cloud[:,3:])

    def addBbox(self, box, points_name=None, box_type="base"):
        if(points_name is None):
            points_name = self.current_point_cloud_name
        addedBox = BoundingBox3D_O3D(box, box_type)
        if "bounding_boxes" not in self.points[points_name]:
            self.points[points_name]["bounding_boxes"] = []
        self.points[points_name]["bounding_boxes"].append(addedBox)

    
    def addBboxes(self, boxes, points_name=None, box_type="base"):
        if(points_name is None):
            points_name = self.current_point_cloud_name
        if "bounding_boxes" not in self.points[points_name]:
            self.points[points_name]["bounding_boxes"] = []
        for box in boxes:
            addedBox = BoundingBox3D_O3D(box)
            self.points[points_name]["bounding_boxes"].append(addedBox)

    def addPcAttribute(self, attribute, points_name=None, attr_name=None):
        if(points_name is None):
            points_name = self.current_point_cloud_name
        assert(points_name in self.points),"points_name must in self.points"
        if "attribute" not in self.points[points_name]:
            self.points[points_name]["attribute"] = {}
        if(attr_name is None):
            attr_name = "attr{num}".format(num=len(self.points[points_name]["attribute"].keys())+1)
        self.points[points_name]["attribute"][attr_name] = attribute
    
    def add_gt_boxes_from_openpcdet(self, gt_boxes, points_name=None)->None:
        """openpcdet的gt框是[N*8]

        Args:
            gt_boxes (torch.Tensor, np.ndarray): [x,y,z,length,width,height,angle,label]
                其中包含全0的包围框,就是不需要的
            pc_index:点云索引

        Returns:
            None
        """
        assert(isinstance(gt_boxes, (torch.Tensor, np.ndarray)))
        if(points_name is None):
            points_name = self.current_point_cloud_name
        if(isinstance(gt_boxes, torch.Tensor)):
            if(gt_boxes.is_cuda):
                gt_boxes = gt_boxes.cpu().numpy()
            else:
                gt_boxes = gt_boxes.numpy()
        #过滤全0包围框
        gt_boxes[:, -1] = gt_boxes[:, -1].astype(int)
        valid_class_mask = (gt_boxes[:,-1] != 0)
        gt_boxes = gt_boxes[valid_class_mask]
        #将gt框的角度转化到open3d坐标系下
        gt_boxes = self.rotate_from_openpcdet2open3d(gt_boxes)
        gt_boxes_added_confidence = (np.zeros([gt_boxes.shape[0], 1])-1)
        gt_boxes = np.concatenate([gt_boxes, gt_boxes_added_confidence], axis=1)
        self.addBboxes(gt_boxes, points_name)

    def add_pred_boxes_from_openpcdet(self, pred_boxes, pred_scores, pred_labels, points_name=None)->None:
        """根据openpcdet的包围框输出显示包围框

        Args:
            pred_boxes (_type_): 预测的包围框:[N,7],[x,y,z,length,width,height,angle]
            pred_scores (_type_): 预测的置信度[N]或者[N,1]
            pred_labels (_type_): 预测的类别[N]
            pc_index (_type_): 点云的索引

        Returns:
            None
        """
        assert(isinstance(pred_boxes, (torch.Tensor, np.ndarray)))
        assert(isinstance(pred_scores, (torch.Tensor, np.ndarray)))
        assert(isinstance(pred_labels, (torch.Tensor, np.ndarray)))
        assert(pred_boxes.shape[0]==pred_scores.shape[0] and pred_boxes.shape[0]==pred_labels.shape[0])
        if(points_name is None):
            points_name = self.current_point_cloud_name
        if(isinstance(pred_boxes, torch.Tensor)):
            if(pred_boxes.is_cuda):
                pred_boxes = pred_boxes.cpu().numpy()
            else:
                pred_boxes = pred_boxes.numpy()
        if(isinstance(pred_scores, torch.Tensor)):
            if(pred_scores.is_cuda):
                pred_scores = pred_scores.cpu().numpy()
            else:
                pred_scores = pred_scores.numpy()
        if(isinstance(pred_labels, torch.Tensor)):
            if(pred_labels.is_cuda):
                pred_labels = pred_labels.cpu().numpy()
            else:
                pred_labels = pred_labels.numpy()
        pred_boxes = self.rotate_from_openpcdet2open3d(pred_boxes)
        #   [N]->[N,1]
        if(len(pred_scores.shape) == 1):
            pred_scores = np.expand_dims(pred_scores, axis=1)
        if(len(pred_labels.shape) == 1):
            pred_labels = np.expand_dims(pred_labels, axis=1)
        pred_boxes = np.concatenate([pred_boxes, pred_labels, pred_scores], axis=1)
        self.addBboxes(pred_boxes, points_name)
    

    @staticmethod
    def rotate_from_openpcdet2open3d(boxes):
        boxes_copied = copy.deepcopy(boxes)
        boxes_copied[:, 6] = np.pi/2 - boxes_copied[:,6]
        return boxes_copied


    def displayPointCloud(self, bg_color=(0,0,0)):
        """显示点云
        """
        assert(len(self.points)>0),"There is no input point cloud."
        self.split_point_cloud()
        data = []
        for pc_name, pc_dict in self.points.items():
            data_dict = {}
            data_dict["name"] = pc_name
            data_dict["points"] = pc_dict["coordinate"]
            data_dict["intensity"] = pc_dict["feature"][:,0]
            if("bounding_boxes" in pc_dict):
                data_dict["bounding_boxes"] = pc_dict["bounding_boxes"]
            if("attribute" in pc_dict):
                for key, value in pc_dict["attribute"].items():
                    data_dict[key] = value
            data.append(data_dict)
        vis = ml3d.vis.Visualizer()
        vis.visualize(data)
    
    def getPointNumInPC(self, points_name=None):
        if(points_name is None):
            points_name = self.current_point_cloud_name
        return self.points[points_name]["points"].shape[0]

    def getPointCloudLength(self):
        return len(self.points)
    
if __name__ == "__main__":
    # pass
    test = {}
    print(len(test))
    pcVisualizer = PCVisualizer()   #test no input
    # pcVisualizer.add_single_point_cloud_from_file(pc_file="../data/000036.bin", points_name = "test")

    # pcVisualizer = PCVisualizer(pc_file="../data/000002.bin")       #test_with_bin_file_in_kitti
    # pcVisualizer.add_single_point_cloud_from_file(pc_file="../data/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151605548192.pcd.bin", points_dim=5)
    # pcVisualizer = PCVisualizer(pc_file="../data/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151605548192.pcd.bin", points_dim=5)       #test_with_bin_file_in_nusc
    # pcVisualizer = PCVisualizer(pc_file="../data/pcd_files/20m/20m.pcd")      #test_with_pcd_file(assert)

    pcVisualizer = PCVisualizer(pc_file="../data/pcd_files/20m/20m.pcd", pc_type = "PointXYZI", points_name = "test")      #test_with_pcd_file
    # shape_0 = pcVisualizer.getPointNumInPC()
    # pcVisualizer.addBbox([1,2,3,4,5,6,7,1,0.9]) #test_added_box
    # pcVisualizer.addBboxes([[1,2,3,4,5,6,7,1,0.9],[10,20,3,4,5,6,7,1,0.9]]) #test_added_boxes
    # pcVisualizer.addPcAttribute(np.random.rand(shape_0, 1))
    # pcVisualizer.addPcAttribute(np.random.rand(shape_0, 1))
    # pcVisualizer.addPcAttribute(np.random.rand(shape_0, 1))
    # pcVisualizer.addPcAttribute(np.random.rand(shape_0, 1))

    pcVisualizer.displayPointCloud()
