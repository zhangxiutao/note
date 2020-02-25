import numpy as np
from functools import cmp_to_key
def pre_order(node):
    if node is not None:
        print(node.val)
        pre_order(node.left)
        pre_order(node.right)

class KDNode():

    def __init__(self,left,right,axis,val):
        self.left = left
        self.right = right
        self.axis = axis
        self.val = val



def create(points,axis=0):
    
    if len(points) == 0:
        return None
    points = sorted(points,key=lambda x: x[axis])
    #print(points)
    axis_child = (axis+1)%(len(points[0]))
    mid_idx = len(points)//2
    mid_point = points[mid_idx]
    points_left = points[:mid_idx]
    points_right = points[mid_idx+1:]
    left = create(points_left,axis_child)
    right = create(points_right,axis_child)
    return KDNode(left,right,axis,mid_point)


point_cloud = np.random.rand(1, 10, 3) - np.random.rand(1, 10, 3)
tree = create(point_cloud)
pre_order(tree)


    