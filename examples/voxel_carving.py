import numpy as np
import open3d as o3d
from pathlib import Path

ROOT_PATH = Path(".").resolve()


def get_armadillo_mesh():
    bunny_path = ROOT_PATH / "assets" / "armadillo.obj"
    mesh = o3d.io.read_triangle_mesh(str(bunny_path))
    mesh.compute_vertex_normals()
    return mesh


def xyz_spherical(xyz):
    # Y-down Z-forward (same as OpenCV)
    # https://github.com/intel-isl/Open3D/issues/1347
    x, y, z = xyz[0], xyz[1], xyz[2]
    r = np.sqrt(x**2 + y**2 + z**2)
    theta = np.arccos(y / r)
    phi = np.arctan2(z, x)
    return [r, theta, phi]


def get_rotation_matrix(theta, phi):
    # dirty
    rot_theta = np.array([[1.0, 0.0, 0.0],
                          [0.0, np.cos(theta), -np.sin(theta)],
                          [0.0, np.sin(theta), np.cos(theta)]])
    rot_phi = np.array([[np.cos(phi), 0, np.sin(phi)],
                        [0.0, 1.0, 0.0],
                        [-np.sin(phi), 0, np.cos(phi)]])
    return rot_phi @ rot_theta


def get_extrinsic(xyz):
    rvec = xyz_spherical(xyz)
    r = get_rotation_matrix(rvec[1], rvec[2])
    t = np.array([0, 0, 2]).transpose()
    trans = np.eye(4)
    trans[:3, :3] = r
    trans[:3, 3] = t
    return trans


def preprocess(model):
    # normalize mesh or point cloud
    min_bound = model.get_min_bound()
    max_bound = model.get_max_bound()
    center = min_bound + (max_bound - min_bound) / 2.0
    vertices = np.array(model.vertices)
    vertices -= center
    scale = np.linalg.norm(max_bound - min_bound) / 2.0
    model.vertices = o3d.utility.Vector3dVector(vertices / scale)
    return model


def voxel_carving(mesh, output_filename, camera_path, cubic_size, voxel_resolution,
                  w=300, h=300, use_depth=True, surface_method="point_cloud"):
    # normalize the input geometry and camera positions
    mesh.compute_vertex_normals()
    mesh = preprocess(mesh)
    camera_sphere = o3d.io.read_triangle_mesh(camera_path)
    camera_sphere = preprocess(camera_sphere)
    # o3d.visualization.draw_geometries([camera_sphere, mesh], width=800, height=600)

    # Create dense voxel grid
    voxel_grid = o3d.geometry.VoxelGrid.create_dense(
        width=cubic_size,
        height=cubic_size,
        depth=cubic_size,
        voxel_size=cubic_size / voxel_resolution,
        origin=[-cubic_size / 2.0, -cubic_size / 2.0, -cubic_size / 2.0]
    )
    # o3d.visualization.draw_geometries([voxel_grid], width=800, height=600, mesh_show_wireframe=True)

    return 0, 0, 0


if __name__ == '__main__':
    mesh = get_armadillo_mesh()
    output_filename = str(ROOT_PATH / "assets" / "voxelized.ply")
    camera_path = str(ROOT_PATH / "assets" / "sphere.ply")
    cubic_size = 2.0

    voxel_resolution = 128.0
    # voxel_resolution = 32.0

    voxel_grid, voxel_carving, voxel_surface = voxel_carving(mesh, output_filename, camera_path, cubic_size, voxel_resolution)

    """
    mesh.scale(1 / np.max(mesh.get_max_bound() - mesh.get_min_bound()), center=mesh.get_center())
    o3d.visualization.draw_geometries([mesh], width=800, height=600)
    """
