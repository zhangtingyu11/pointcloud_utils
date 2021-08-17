import open3d as o3d
import numpy as np
from bbox_utils import BoundingBox_3d
import math

class PointCloud():
	def __init__(self, channel_num = 3, filename = None, points = None) -> None:
		"""[初始化点云类]

		Args:
		    channel_num (int, optional): [点云的维度，目前kitti数据集需要设置为4，nuscenes数据集需要设置为]. Defaults to 3.
		    filename ([type], optional): [读取的文件名]. Defaults to None.
		    points ([type], optional): [可以直接读取点云]. Defaults to None.
		"""
		self.channel_num = channel_num
		if(filename != None):
			self.read_pc_from_file(filename)
		if(points != None):
			self.pc_numpy = points
		self.create_vis()

		

	def read_pc_from_file(self, filename:str):
		"""[从文件中读取点云]

		Args:
		    filename (str): [文件名]
		"""
		if filename.endswith('.pcd'):
			self.read_pcd_file(filename)
		elif filename.endswith('bin'):
			self.read_bin_file(filename)

	def read_pcd_file(self, pcd_filename):
		"""[目前读取pcd文件，点云的维度只能是3]

		Args:
		    pcd_filename ([str]): [pcd文件路径]
		"""

		pts = []
		f = open(pcd_filename, 'r')
		data = f.readlines()

		f.close()
		line = data[9]
		# print line
		line = line.strip('\n')
		i = line.split(' ')
		pts_num = eval(i[-1])
		for line in data[11:]:
			line = line.strip('\n')
			point_attr = line.split(' ')
			a_float_m = map(float, point_attr)
			pts.append(list(a_float_m))
		self.pc_numpy = np.asarray(pts, dtype=np.float32)
		self.open_cloud = self.numpy_to_open3d()


	def read_bin_file(self, bin_filename):
		self.pc_numpy = np.fromfile(bin_filename, dtype=np.float32).reshape(-1, self.channel_num)
		self.open_cloud = self.numpy_to_open3d()

	def numpy_to_open3d(self):
		open_cloud = o3d.geometry.PointCloud()
		open_cloud.points = o3d.utility.Vector3dVector(self.pc_numpy[:,0:3])
		return open_cloud

	def create_vis(self):
		self.vis = o3d.visualization.Visualizer()
		self.vis.create_window()

	def destroy_vis(self):
		self.vis.destroy_window()

	def display_pc(self, bbox_array = None):
		self.vis.add_geometry(self.open_cloud)
		render_option = self.vis.get_render_option()
		render_option.point_size = 2
		render_option.background_color = np.asarray([0, 0, 0])
	
		self.vis.run()
		self.destroy_vis()

	def draw_3dboxes(self, boxes, color = [0,1,0]):
		for box in boxes:
			self.draw_3dbox(box, color)


	def draw_3dbox(self, bbox:BoundingBox_3d, color = [0,1,0]):
		"""[基于点云画3D包围框]

		Args:
		    bbox (np.BoundingBox_3d): [3D包围框的实例]
		1 -------- 0
		/|         /|
		2 -------- 3 .
		| |       | |
		. 5 -------- 4
		|/        |/
		6 -------- 7
		"""
		points_box = bbox.get_3d_corners()
		lines_box = np.array([[0, 1], [1, 2], [2, 3], [3, 7], [4, 5], [5, 6], [6, 7], [3, 0],
					[0, 4], [1, 5], [2, 6], [4, 7]])
		colors = np.array([color for j in range(len(lines_box))])
		line_set = o3d.geometry.LineSet()
		line_set.points = o3d.utility.Vector3dVector(points_box)
		line_set.lines = o3d.utility.Vector2iVector(lines_box)
		line_set.colors = o3d.utility.Vector3dVector(colors)
		self.vis.add_geometry(line_set)
		
if __name__ == '__main__':
	filename = 'data/TCC12.pcd'
	PCL = PointCloud(4, filename)
	f = open('./label/PC_0729_3.txt', 'r')
	bbox_list = []
	line = f.readline().rstrip()
	while line:
		label, occlusion, length, width, height, x, y, z, theta, angle = line.split('\t')
		angle = math.radians(float(angle))
		bbox = BoundingBox_3d(float(length), float(width), 
			float(height), float(x), float(y), float(z), float(angle))
		bbox_list.append(bbox)
		line = f.readline().rstrip()
	PCL.draw_3dboxes(bbox_list, color=[1,0,0])
	#just for test
	# box = BoundingBox_3d(5.15193, 1.89721, 1.81758, 10, 10, -1.16059, math.radians(0))
	# PCL.draw_3dbox(box, transform_to_open3d=True)
	PCL.display_pc()

