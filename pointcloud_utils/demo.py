from pointcloud import PointCloud
from evaluation import Evaluation_3d

pc = PointCloud(channel_num=4, filename='../data/TCC12.pcd')
pc.create_vis()
pred_boxes = pc.draw_3dboxes_from_txt('../data/PC_0729_3_pred.txt')
gt_boxes = pc.draw_3dboxes_from_txt('../data/PC_0729_3.txt', color=[1, 0, 1])
eval = Evaluation_3d(pred_boxes, gt_boxes, 0.5)
recall = eval.compute_recall()
pc.display_pc()
