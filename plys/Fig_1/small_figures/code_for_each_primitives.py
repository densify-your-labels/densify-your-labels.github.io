from helper_ply import read_ply, write_ply
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
import pprint
from pathlib import Path
import matplotlib

if __name__ == '__main__':
    data = read_ply('Path2Primitives')
    coords = np.vstack((data['x'], data['y'], data['z'])).T.copy()
    colors = np.vstack((data['red'], data['green'], data['blue'])).T.copy()
    colors = colors/255.0
    labels = data['class']
    primitive_ids = np.unique(labels)
    pcd = o3d.geometry.PointCloud()
    # o3d.visualization.draw_geometries([pcd])
    # # Show one primitive
    for i in range(len(primitive_ids)):
        color_copy = np.copy(colors)
        show_id = primitive_ids[i]
        mask = labels != show_id
        color_copy[mask] = [0.95, 0.95, 0.95]
        pcd.points = o3d.utility.Vector3dVector(coords)
        pcd.colors = o3d.utility.Vector3dVector(color_copy)
        o3d.visualization.draw_geometries([pcd])