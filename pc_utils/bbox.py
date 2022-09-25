from typing import List
from open3d._ml3d.vis.boundingbox import BoundingBox3D
import numpy as np
import math


label_to_num ={
    "car":1
}

num_to_label ={
    1:"car"
}


class BoundingBox3D_O3D(BoundingBox3D):
    def __init__(self, box, box_type="base"):
        """_summary_

        Args:
            box (_type_): [x,y,z,length,width,height,angle,label,confidence]
            box_type (str, optional): _description_. Defaults to "base".
        """
        assert(isinstance(box, List) or isinstance(box, np.ndarray)),\
            "box_array should be list or ndarray.But got {box_type}".format(box_type=type(box))
        if(box_type == "base"):
            x,y,z,length,width,height,angle,label_num, conf = box

        center = [x,y,z]
        sin_angle_front = math.sin(angle)
        cos_angle_front = math.cos(angle)
        sin_angle_left = math.sin(angle+math.pi/2)
        cos_angle_left = math.cos(angle+math.pi/2)

        front = [sin_angle_front, cos_angle_front, 0]
        up = [0,0,1]
        left = [sin_angle_left, cos_angle_left, 0]
        size = [width, height, length]
        label_class = self.num2label(label_num)
        confidence = conf
        super().__init__(center, front, up, left, size, label_class, confidence, show_confidence=True)

    def num2label(self, num):
        return num_to_label[num]
