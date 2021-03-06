import numpy as np
import open3d as o3d
from pathlib import Path

ROOT_PATH = Path(".").resolve()


def get_bunny_mesh():
    bunny_path = ROOT_PATH / "assets" / "bunny.ply"
    mesh = o3d.io.read_triangle_mesh(str(bunny_path))
    mesh.compute_vertex_normals()
    return mesh


if __name__ == '__main__':
    mesh = get_bunny_mesh()

    # Fit to unit cube
    mesh.scale(1 / np.max(mesh.get_max_bound() - mesh.get_min_bound()), center=mesh.get_center())
    o3d.visualization.draw_geometries([mesh], width=800, height=600)

    # Voxelization
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh,
                                                                  voxel_size=0.05)
    o3d.visualization.draw_geometries([voxel_grid], width=800, height=600)
