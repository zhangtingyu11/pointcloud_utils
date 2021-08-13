import open3d as o3d

class PointCloud():
	def __init__(self) -> None:
		pass
	
	@staticmethod
	def read_pc_from_file(self, filename:str):
		if filename.endswith('.pcd'):
			pcd = o3d.io.read_point_cloud(filename)
			print(pcd)
   
