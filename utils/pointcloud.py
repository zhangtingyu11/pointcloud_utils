import open3d as o3d
import numpy as np
import pcl

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
		pcd = pcl.load(filename)
		self.pc_numpy = np.asarray(pcd)
		self.channel_num = 3
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
		self.create_vis()
		self.vis.add_geometry(self.open_cloud)
		points_box = np.array([[0, 0, 0],[1,0,0],[1,3,0],[0,3,0],[0,3,3],[0,0,3],[1,0,3],[1,3,3]])
		lines_box = np.array([[0, 1], [1, 2], [2, 3], [3, 4], [4, 5], [5, 6], [6, 7], [3, 0],
					[7, 4], [0, 5], [1, 6], [2, 7]])
		colors = np.array([[0, 1, 0] for j in range(len(lines_box))])
		line_set = o3d.geometry.LineSet()
		line_set.points = o3d.utility.Vector3dVector(points_box)
		line_set.lines = o3d.utility.Vector2iVector(lines_box)
		line_set.colors = o3d.utility.Vector3dVector(colors)
		self.vis.add_geometry(line_set)

		render_option = self.vis.get_render_option()
		render_option.point_size = 2
		render_option.background_color = np.asarray([0, 0, 0])
	
		self.vis.run()
		
		self.destroy_vis()

	def draw_3dboxes(self, ):


	def draw_3dbox(self, bbox_array:np.array):
		"""[基于点云画3D包围框]

		Args:
		    bbox_array (np.array): [3D包围框的8个点，需要按照以下顺序排列，为8*3的np.array]
		1 -------- 0
		/|             /|
		2 -------- 3 .
		| |            | |
		. 5 -------- 4
		|/             |/
		6 -------- 7
		"""
		

		
		lines_box = np.array([[0, 1], [1, 2], [0, 3], [2, 3], [4, 5], [4, 7], [5, 6], [6, 7],
			[0, 4], [1, 5], [2, 6], [3, 7]])
		colors = np.array([[0, 1, 0] for j in range(len(lines_box))])
		line_set = o3d.geometry.LineSet()
		line_set.points = o3d.utility.Vector3dVector(points_box)
		line_set.lines = o3d.utility.Vector2iVector(lines_box)
		line_set.colors = o3d.utility.Vector3dVector(colors)
		point_cloud = o3d.geometry.PointCloud()
		point_cloud.points = o3d.utility.Vector3dVector(pc[:,:3])
		# generateBox(point_cloud)
		custom_draw_geometry(point_cloud, line_set)
		
if __name__ == '__main__':
	filename = 'data/n008-2018-08-01-15-16-36-0400__LIDAR_TOP__1533151605548192.pcd.bin'
	PCL = PointCloud(5, filename)
	PCL.display_pc()()

