from pointcloud_utils.bbox_utils import BoundingBox_2d
from pointcloud_utils.bbox_utils import BoundingBox_3d
import sys

class Evaluation_2d():
        def __init__(self, pred_boxes, gt_boxes, iou_thresh = 0.7):
                self.pred_boxes = pred_boxes
                self.pred_boxes.sort(key = lambda x:x.score, reverse=True)
                self.gt_boxes = gt_boxes
                self.gt_boxes.sort(key = lambda x:x.score, reverse=True)
                self.iou_thresh = iou_thresh
        
        def compute_TP_FP(self):
                #用于记录gt_boxes是否有匹配的box
                gt_flag = [0]*len(self.gt_boxes)
                for i in range(len(self.pred_boxes)):
                        max_iou = sys.float_info.min
                        max_index = sys.float_info.min
                        for j in range(len(self.gt_boxes)):
                                iou = BoundingBox_2d.compute_2d_iou(self.pred_boxes[i], self.gt_boxes[j]) 
                                if(iou < self.iou_thresh):
                                        pass
                                else:
                                        if(max_iou < iou):
                                                max_iou = iou
                                                max_index = j
                        if(max_index == sys.float_info.min):
                                pass
                        else:
                                gt_flag[max_index] = 1  
                
                TP_num = sum(gt_flag)
                FP_num = len(self.pred_boxes) - TP_num
                for i in range(len(gt_flag)):
                        if(gt_flag[i] == 0):
                                gt_flag[i] = False
                        else:
                                gt_flag[i] = True
                return TP_num, FP_num, gt_flag
        
        def compute_recall(self):
                TP,FP,gt_flag = self.compute_TP_FP()
                print("TP num:{}".format(TP))
                print("FP num:{}".format(FP))
                recall = TP/len(self.gt_boxes)
                print("Recall:{}".format(recall))
                return recall,gt_flag

        def compute_precision(self):
                TP,FP,gt_flag = self.compute_TP_FP()
                print("TP num:{}".format(TP))
                print("FP num:{}".format(FP))
                precision = TP/len(self.pred_boxes)
                print("Precision:{}".format(precision))
                return precision,gt_flag

class Evaluation_3d():
        def __init__(self, pred_boxes, gt_boxes, iou_thresh = 0.7):
                self.pred_boxes = pred_boxes
                self.gt_boxes = gt_boxes
                self.iou_thresh = iou_thresh
        
        def compute_TP_FP(self):
                pred_boxes_2d = []
                for box in self.pred_boxes:
                        pred_boxes_2d.append(box.get_bev_2d_box())
                gt_boxes_2d = []
                for box in self.gt_boxes:
                        gt_boxes_2d.append(box.get_bev_2d_box())
                eval = Evaluation_2d(pred_boxes_2d, gt_boxes_2d, self.iou_thresh)
                return eval.compute_TP_FP()
        
        def compute_recall(self):
                TP,FP,gt_flag = self.compute_TP_FP()
                print("TP num:{}".format(TP))
                print("FP num:{}".format(FP))
                recall = TP/len(self.gt_boxes)
                print("Recall:{}".format(recall))
                return TP/len(self.gt_boxes),gt_flag

        def compute_precision(self):
                TP,FP,gt_flag = self.compute_TP_FP()
                print("TP num:{}".format(TP))
                print("FP num:{}".format(FP))
                recall = TP/len(self.gt_boxes)
                print("Recall:{}".format(recall))
                return TP/len(self.pred_boxes),gt_flag
