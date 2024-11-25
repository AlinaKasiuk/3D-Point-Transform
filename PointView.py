# -*- coding: utf-8 -*-
"""
Created on Mon Nov 18 15:51:02 2024
@author: Alina
"""

import numpy as np
import open3d as o3d
import pandas as pd
import ast

# Step 1: Read positions from the CSV file
csv_file = "PointCloud/exp3/table.csv"  # Replace with your actual file path
data = pd.read_csv(csv_file, sep=";")  # Use `;` as the delimiter
positions_raw = data["Position"]  # Extract the 'Position' column
positions = np.array([ast.literal_eval(pos) for pos in positions_raw])  # Convert string to list of coordinates

# Swap axes and scale coordinates if needed
positions = np.array([ast.literal_eval(pos) for pos in positions_raw])  # Original parsing
positions[:, [1, 2]] = positions[:, [2, 1]]  # Swap y and z to fix vertical movement
scale_factor = 1.0  # Adjust scale if needed
positions *= scale_factor

# Step 2: Load the existing point cloud
pcd = o3d.io.read_point_cloud("PointCloud/exp3/lidar/pointcloud2.ply")

# Step 3: Ensure the point cloud loads successfully
if not pcd.has_points():
    raise RuntimeError("Failed to load the point cloud from the specified file.")
print(f"Number of points before: {len(pcd.points)}")

# Step 4: Convert existing points and colors to NumPy arrays
points = np.asarray(pcd.points)

if pcd.has_colors():
    colors = np.asarray(pcd.colors)
else:
    # If the point cloud has no colors, initialize all points as black
    colors = np.zeros_like(points)

# Step 5: Append new points and colors
#new_points = positions  # These are the points from the CSV
#new_colors = np.tile([1.0, 0.0, 0.0], (len(new_points), 1))  # Red color for all new points

new_point = np.array([[0.0, 0.0, 0.0]])  # Example coordinates for the new point
new_color = np.array([[1.0, 0.0, 0.0]])

points = np.vstack((points, new_point))  # Add new points
colors = np.vstack((colors, new_color))  # Add corresponding colors

# Step 6: Update the point cloud
pcd.points = o3d.utility.Vector3dVector(points)
pcd.colors = o3d.utility.Vector3dVector(colors)

# Step 7: Verify the update
print(f"Number of points after: {len(pcd.points)}")

# Step 8: Create a voxel grid for visualization
voxel_size = 0.05  # Adjust for better resolution
voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=voxel_size)

# Step 9: Visualize the updated point cloud or voxel grid
o3d.visualization.draw_geometries([pcd], window_name="Updated Point Cloud")
# Optionally, visualize the voxel grid instead:
# o3d.visualization.draw_geometries([voxel_grid], window_name="Voxel Grid Visualization")

