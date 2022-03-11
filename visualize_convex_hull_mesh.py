#!/usr/bin/env python3
"""
Author : Emmanuel Gonzalez
Date   : 2022-03-11
Purpose: Visualization of convex hull and/or mesh
"""

import argparse
import os
import sys
import glob 
import numpy as np 
import pandas as pd 
import open3d as o3d
import matplotlib.pyplot as plt

# --------------------------------------------------
def get_args():
    """Get command-line arguments"""

    parser = argparse.ArgumentParser(
        description='Convex hull & mesh visualization',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument('point_cloud',
                        help='Point cloud to visualize',
                        metavar='str',
                        type=str)
    
    parser.add_argument('-c',
                        '--convex_hull',
                        help='Visualize convex hull',
                        action='store_true')

    parser.add_argument('-m',
                        '--mesh',
                        help='Visualize mesh',
                        action='store_true')

    parser.add_argument('-v',
                        '--voxel_size',
                        help='Voxel size to use for downsampling',
                        type=float,
                        required=False)

    parser.add_argument('-d',
                        '--depth',
                        help='Depth value for poisson reconstruction',
                        type=int,
                        default=9)

    parser.add_argument('-w',
                        '--width',
                        help='Width value for poisson reconstruction',
                        type=int,
                        default=0)

    parser.add_argument('-s',
                        '--scale',
                        help='Scale value for poisson reconstruction',
                        type=float,
                        default=1.1)     

    parser.add_argument('-l',
                        '--linear_fit',
                        help='Scale value for poisson reconstruction',
                        type=bool,
                        default=False)                     

    return parser.parse_args()


# --------------------------------------------------
def get_paths(directory):

    pcd_list = []

    for root, dirs, files in os.walk(directory):
        for name in files:
            if 'final.ply' in name:
                pcd_list.append(os.path.join(root, name))

    if not pcd_list:

        raise Exception(f'ERROR: No compatible point clouds found in {directory}.')

    return pcd_list


# --------------------------------------------------
def open_pcd(pcd_path):

    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f':: Opening "{pcd_path}".')
    pcd.estimate_normals()
    print(':: Estimating normals.')
    pcd.normalize_normals()
    print(':: Normalizing estimated normals.')
    
    return pcd


# --------------------------------------------------
def downsample_pcd(pcd, voxel_size):

    down_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)

    return down_pcd


# --------------------------------------------------
def visualize_pcd(pcd, extra=None):
    if extra:
        o3d.visualization.draw_geometries([pcd, extra])
    else:    
        o3d.visualization.draw_geometries([pcd])


# --------------------------------------------------
def calculate_convex_hull_volume(pcd):
    hull, _ = pcd.compute_convex_hull()
    hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
    hull_ls.paint_uniform_color((1, 0, 0))
    hull_volume = hull.get_volume()
    return hull_volume, hull_ls


# --------------------------------------------------
def calculate_oriented_bb_volume(pcd):

    obb_vol = pcd.get_oriented_bounding_box().volume()

    return obb_vol


# --------------------------------------------------
def calculate_axis_aligned_bb_volume(pcd):

    abb_vol = pcd.get_axis_aligned_bounding_box().volume()

    return abb_vol


# --------------------------------------------------
def convert_point_cloud_to_array(pcd):
    
    # Convert point cloud to a Numpy array
    pcd_array = np.asarray(pcd.points)
    pcd_array  = pcd_array.astype(float)

    return pcd_array


# --------------------------------------------------
def get_min_max(pcd):
    
    max_x, max_y, max_z = pcd.get_max_bound()
    min_x, min_y, min_z = pcd.get_min_bound()

    return max_x, max_y, max_z, min_x, min_y, min_z


# --------------------------------------------------
def remove_statistical_outliers(pcd, nb_neighbors=10, std_ratio=2):

    pcd =pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return pcd 


# --------------------------------------------------
def crop_point_cloud(pcd, bbox_coords, max_z, min_z):

    bounding_polygon = np.array(bbox_coords).astype('float64')

    vol = o3d.visualization.SelectionPolygonVolume()
    vol.orthogonal_axis = "Z"
    vol.axis_max = max_z
    vol.axis_min = min_z
    vol.bounding_polygon = o3d.utility.Vector3dVector(bounding_polygon)

    crop = vol.crop_point_cloud(pcd)

    return crop


# --------------------------------------------------
def poisson_mesh(pcd, depth, width, scale, linear_fit):
    print(':: Running Poisson surface reconstruction')
    with o3d.utility.VerbosityContextManager(
            o3d.utility.VerbosityLevel.Debug) as cm:
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=depth, width=width, scale=scale, linear_fit=linear_fit)

    print(':: Calculating densities')
    densities = np.asarray(densities)
    density_colors = plt.get_cmap('plasma')(
        (densities - densities.min()) / (densities.max() - densities.min()))
    density_colors = density_colors[:, :3]
    density_mesh = o3d.geometry.TriangleMesh()
    density_mesh.vertices = mesh.vertices
    density_mesh.triangles = mesh.triangles
    density_mesh.triangle_normals = mesh.triangle_normals
    density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)

    print(':: Removing low density vertices')
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)

    return mesh


# --------------------------------------------------
def main():
    """Visualize point cloud here"""

    args = get_args()
    pcd_path = args.point_cloud
    
    # Open point cloud 
    pcd = open_pcd(pcd_path)

    # Down sample point cloud
    if args.voxel_size:
        pcd = downsample_pcd(pcd, voxel_size=0.05)

    # Calculate convex hull
    if args.convex_hull:
        hull_volume, hull_ls = calculate_convex_hull_volume(pcd)
        visualize_pcd(pcd, extra=hull_ls)

    # Generate mesh from point cloud
    if args.mesh:
        pcd = pcd.paint_uniform_color([0, 1, 0])
        mesh = poisson_mesh(pcd, 
                            depth=args.depth, 
                            width=args.width, 
                            scale=args.scale, 
                            linear_fit=args.linear_fit)
                            
        visualize_pcd(mesh)


# --------------------------------------------------
if __name__ == '__main__':
    main()
