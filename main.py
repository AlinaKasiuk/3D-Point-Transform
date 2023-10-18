# -*- coding: utf-8 -*-
"""
Created on Wed Oct 18 11:08:22 2023

@author: Alina
"""
import numpy as np
import open3d as o3d

point_cloud = np.loadtxt('bunnyStatue.txt', delimiter=' ')
points = point_cloud[:,:3]
#points = np.random.rand(1000, 3)
#a = np.reshape(point_cloud[:,5], (point_cloud.shape[0],1))
#b = np.zeros([point_cloud.shape[0], 2])
#colors = np.append(b,a, axis=1)

for i in range (10):
    new_points = (np.random.rand(1000, 3)-0.5)*2
    points = np.append(points,new_points, axis=0)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)
#pcd.colors = o3d.utility.Vector3dVector(colors)



# Create a voxel grid from the point cloud with a voxel_size of 0.01
voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=0.001)

# Initialize a visualizer object
vis = o3d.visualization.Visualizer()
# Create a window, name it and scale it
vis.create_window(window_name='Bunny Visualize', width=800, height=600)

# Add the voxel grid to the visualizer
vis.add_geometry(voxel_grid)

# We run the visualizater
vis.run()
# Once the visualizer is closed destroy the window and clean up
vis.destroy_window()
