import numpy as np
import open3d as o3d
import pandas as pd
import ast  # For safely evaluating string representations of lists
import matplotlib.pyplot as plt  # For color mapping

# Load the CSV file
csv_file = "exp6/table.csv"  # Replace with your actual file path
data = pd.read_csv(csv_file, sep=";")  # Use ';' as delimiter

# Load the first point cloud (pointcloud0.ply)
pointcloud_file = "exp6/lidar/pointcloud0.ply"
pcd = o3d.io.read_point_cloud(pointcloud_file)
points = np.asarray(pcd.points)

# If the point cloud has no colors, initialize them as black
if pcd.has_colors():
    colors = np.asarray(pcd.colors)
else:
    colors = np.zeros_like(points)

# Extract the transformation matrix from the first row in the CSV
transformation_matrix_raw = data["Transformation matrix"].iloc[0]  # Take the first matrix
transformation_matrix_raw = transformation_matrix_raw.replace('[[', '').replace(']]', '')  # Clean the string
rows = transformation_matrix_raw.split('],[')  # Split by inner brackets
rows = [r.replace(']', '').replace('[', '') for r in rows]  # Clean each row

# Convert the string rows into a numpy array
transformation_matrix = np.array([list(map(float, r.split())) for r in rows]).reshape(4, 4)

# Add homogeneous coordinates (convert to 4D) to the point cloud
points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))  # Shape: (N, 4)

# Apply the transformation matrix to the points
transformed_points = (transformation_matrix.T @ points_homogeneous.T).T[:, :3]

# Create a new point cloud for the transformed points
transformed_pcd = o3d.geometry.PointCloud()
transformed_pcd.points = o3d.utility.Vector3dVector(transformed_points)
transformed_pcd.colors = o3d.utility.Vector3dVector(colors)

# Create a color map based on the "Mean wheel torque"
mean_wheel_torque = data["Mean wheel torque"]  # Extract Mean wheel torque from the CSV

# Normalize the mean wheel torque to [0, 1] range for coloring
min_torque = mean_wheel_torque.min()
max_torque = mean_wheel_torque.max()
normalized_torque = (mean_wheel_torque - min_torque) / (max_torque - min_torque)

# Use matplotlib to generate colors based on the normalized torque
cmap = plt.get_cmap("viridis")  # You can choose other colormaps like "jet", "plasma", etc.
colors_for_spheres = cmap(normalized_torque)[:, :3]  # Extract RGB values (ignore alpha channel)

# Create a larger red sphere at each position from the CSV and apply the colors based on torque
red_spheres = []
for i in range(len(data)):  # Iterate through all the rows to get all the positions
    position_raw = data["Position"].iloc[i]
    position = ast.literal_eval(position_raw)  # Convert the string representation of the list into a list
    position= [position[0],position[1],position[2]]
    # Create a sphere and move it to the position
    radius = 0.1  # Adjust the radius of the red point (larger than the normal points)
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    mesh_sphere.translate(position)  # Move the sphere to the position
    mesh_sphere.paint_uniform_color(colors_for_spheres[i])  # Apply color based on Mean wheel torque
    red_spheres.append(mesh_sphere)  # Add the sphere to the list

coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
    size=0.5, # Adjust the size of the axes for better visibility
    origin=[0, 0, 0] # Set the origin of the coordinate frame
)

# Visualize the transformed point cloud, red positions, and the coordinate frame
o3d.visualization.draw_geometries(red_spheres + [coordinate_frame],
                                  window_name="PointCloud with Colored Positions and Axes")

# Visualize the transformed point cloud and all the red positions with color
#o3d.visualization.draw_geometries([transformed_pcd] + red_spheres, window_name="PointCloud with Colored Positions")
