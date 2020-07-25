import numpy as np
import open3d as o3d
from pathlib import Path

ROOT_PATH = Path(".").resolve()


def get_bunny_mesh():
    bunny_path = ROOT_PATH / "assets" / "unchan_pink.obj"
    mesh = o3d.io.read_triangle_mesh(str(bunny_path))
    mesh.compute_vertex_normals()
    return mesh


if __name__ == '__main__':
    mesh = get_bunny_mesh()

    # Fit to unit cube
    N = 2000
    pcd = mesh.sample_points_poisson_disk(N)
    pcd.scale(1 / np.max(pcd.get_max_bound() - pcd.get_min_bound()), center=pcd.get_center())
    pcd.colors = o3d.utility.Vector3dVector(np.random.uniform(0, 1, size=(N, 3)))
    o3d.visualization.draw_geometries([pcd], width=800, height=600)

    # Voxelization
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(pcd, voxel_size=0.05)
    o3d.visualization.draw_geometries([voxel_grid], width=800, height=600)
