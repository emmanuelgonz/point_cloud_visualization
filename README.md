# Point Cloud Visualization

This script allows you to visualize convex hull and/or mesh objects calculated using Open3D (v. 0.15.2). 

## Arguments

- Positional 
  - 'point_cloud'
    - Path to the point cloud to be visualized
- Flags 
  - '-c', '--convex_hull'
    - Optional
    - Visualize convex hull
  - '-m', '--mesh' > 
    - Optional
    - Visualize mesh
  - '-v', '--voxel_size' > 
    - Optional, float
    - Voxel size to use for downsampling. If not given, point cloud will not be downsampled. 
  - 'd', '--depth'
    - Default=9
    - Depth for Poisson surface reconstruction.
  - '-w', '--width'
    - Default=0
    - Width for Poisson surface reconstruction.
  - '-s', '--scale'
    - Default=1.1
    - Scale for Poisson surface reconstruction.
  - '-l', '--linear_fit'
    - Default=False
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