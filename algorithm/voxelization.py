import numpy as np
import open3d as o3d
from statistics import mean
def downsample(x_num,y_num,z_num,x_diff,y_diff,z_diff,x_start,y_start,z_start,pc):
    x_length=x_diff/x_num
    y_length=y_diff/y_num
    z_length=z_diff/z_num
    x_voxel = 0
    y_voxel = 0
    z_voxel = 0
    pc_voxel = []
    voxel_cut_set = []
    clusters = []
    mean_point_clusters = []
    for p in pc:
        x = p[0]
        y = p[1]
        z = p[2]
        x_voxel = ((x-x_start)//x_length)+1
        y_voxel = ((y-y_start)//y_length)+1
        z_voxel = ((z-z_start)//z_length)+1
        pc_voxel.append([x_voxel,y_voxel,z_voxel])
    found_flag = False
    for p_idx,p_voxel in enumerate(pc_voxel):
        for cluster_idx,coordinate_voxel in enumerate(voxel_cut_set):
            if p_voxel == coordinate_voxel:
                clusters[cluster_idx].append(pc[p_idx])
                found_flag = True
                break
        if found_flag == False:
            voxel_cut_set.append(p_voxel)
            clusters.append([])
            clusters[len(voxel_cut_set)-1].append(pc[p_idx])
    for c in clusters:
        p_mean = []
        for i in range(3):
            p_mean.append(np.mean(np.array(c).transpose()[i])) #np.array(c)[:,i]
        mean_point_clusters.append(p_mean)
    print(np.asarray(mean_point_clusters).shape)
    return mean_point_clusters


    

def drawGrid(x_num,y_num,z_num,x_diff,y_diff,z_diff,x_start,y_start,z_start,things):
    things_array = []
    lines_array = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    x_length=x_diff/x_num
    y_length=y_diff/y_num
    z_length=z_diff/z_num
    for z in range(z_num):
        for y in range(y_num):
            for x in range(x_num):
                xmin = x_start + x*x_length
                xmax = x_start + (x+1)*x_length
                ymin = y_start + y*y_length
                ymax = y_start + (y+1)*y_length
                zmin = z_start + z*z_length
                zmax = z_start + (z+1)*z_length

                vertex_array = [[xmin,ymin,zmin],
                        [xmin,ymin,zmax],
                        [xmin,ymax,zmin],
                        [xmin,ymax,zmax],
                        [xmax,ymin,zmin],
                        [xmax,ymin,zmax],
                        [xmax,ymax,zmin],
                        [xmax,ymax,zmax]]

                cubic_line_set = o3d.geometry.LineSet(
                    points=o3d.utility.Vector3dVector(vertex_array),
                    lines=o3d.utility.Vector2iVector(lines_array),
                )
                things_array.append(cubic_line_set)
    things_array = things_array+things
    return things_array
    #o3d.visualization.draw_geometries(things_array)

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("E:\\data\\tutorials\\ism_test_cat.pcd")

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = pcd.voxel_down_sample(voxel_size=0.05)

    pt_np = np.asarray(downpcd.points)
    xmin = min(pt_np.transpose()[0])
    xmax = max(pt_np.transpose()[0])
    ymin = min(pt_np.transpose()[1])
    ymax = max(pt_np.transpose()[1])
    zmin = min(pt_np.transpose()[2])
    zmax = max(pt_np.transpose()[2])

    boundingbox = [[xmin,ymin,zmin],
                   [xmin,ymin,zmax],
                   [xmin,ymax,zmin],
                   [xmin,ymax,zmax],
                   [xmax,ymin,zmin],
                   [xmax,ymin,zmax],
                   [xmax,ymax,zmin],
                   [xmax,ymax,zmax]]
    boundingbox_lines = [
        [0, 1],
        [0, 2],
        [1, 3],
        [2, 3],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]
    boundingbox_line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(boundingbox),
        lines=o3d.utility.Vector2iVector(boundingbox_lines),
    )
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=50, origin=[-2, -2, -2]) #xyz,red,green,blue

    x_diff = (xmax-xmin)
    y_diff = (ymax-ymin)
    z_diff = (zmax-zmin)
    things = []
    #things.append(downpcd)
    things.append(mesh_frame)


    line_set = drawGrid(5,5,5,x_diff,y_diff,z_diff,xmin,ymin,zmin,things)
    things=things+line_set
    xyz=downsample(5,5,5,x_diff,y_diff,z_diff,xmin,ymin,zmin,pt_np)
    pcd_downsampled = o3d.geometry.PointCloud()
    pcd_downsampled.points = o3d.utility.Vector3dVector(xyz)
    things.append(pcd_downsampled)
    o3d.visualization.draw_geometries(things)