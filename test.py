import numpy as np
import open3d as o3d
import pandas as pd
import ast  # For safely evaluating string representations of lists
import matplotlib.pyplot as plt  # For color mapping

# Load the CSV file
csv_file = "exp6/table.csv"  # Replace with your actual file path
data = pd.read_csv(csv_file, sep=";")  # Use ';' as delimiter

#Load the first point cloud (pointcloud0.ply)
pointcloud_file = "exp6/lidar/pointcloud0.ply"
pcd = o3d.io.read_point_cloud(pointcloud_file)
points = np.asarray(pcd.points)

# Extract the transformation matrix from the first row in the CSV
transformation_matrix_raw = data["Transformation matrix"].iloc[30]  # Take the first matrix
position_raw = data["Position"].iloc[0]
rows = position_raw .split(',')  # Split by inner brackets
points = [r.replace(']', '').replace('[', '') for r in rows]



points = np.array([list(map(float, r.split())) for r in points]).reshape(1, 3)

print(transformation_matrix_raw)
print(position_raw)
points =np.array([1,2,3]).reshape(1, 3)
print(points)

points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))  # Shape: (N, 4)

print(points_homogeneous )

rows = transformation_matrix_raw.split('],[')  # Split by inner brackets
rows = [r.replace(']', '').replace('[', '') for r in rows]  # Clean each row
# Convert the string rows into a numpy array
transformation_matrix = np.array([list(map(float, r.split())) for r in rows]).reshape(4, 4)
print(transformation_matrix)

print(transformation_matrix @ points_homogeneous.T)