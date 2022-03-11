# Point Cloud Visualization

This script allows you to visualize convex hull and/or mesh objects calculated using Open3D (v. 0.15.2). 

## Arguments

- Positional 
  - 'point_cloud'
    - Path to the point cloud to be visualized
- Flags 
  - '-c' '--convex_hull' > optional
    - Visualize convex hull
  - '-m' '--mesh' > optional
    - Visualize mesh
  - '-v' '--voxel_size' > float, optional
    - Voxel size to use for downsampling. If not given, point cloud will not be downsampled. 
  - 'd' '--depth' > float, default=9
    - Depth for Poisson surface reconstruction.
  - '-w' '--width' > float, default=0
    - Width for Poisson surface reconstruction.
  - '-s' '--scale' > float, default=1.1
    - Scale for Poisson surface reconstruction.
  - '-l' '--linear_fit' > bool, default=False
    - Linear fit for Poisson surface reconstruction.
    
## Example 

To visualize the sample point cloud, run the following command: 

### Visualize convex hull

```bash
./visualize_convex_hull_mesh.py -c final.ply
```

### Visualize mesh 

```bash 
./visualize_convex_hull_mesh.py -m final.ply
```