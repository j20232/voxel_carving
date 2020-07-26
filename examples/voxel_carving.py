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


class VoxelCarving():
    def __init__(self, camera_path: str, cubic_size: int, voxel_resolution: int):
        camera_pos = o3d.io.read_triangle_mesh(camera_path)
        self.camera_pos = self._normalize_geometry(camera_pos)
        # o3d.visualization.draw_geometries([camera_pos, mesh], width=800, height=600)

        # Create dense voxel grid
        self.voxel_grid = o3d.geometry.VoxelGrid.create_dense(
            width=cubic_size,
            height=cubic_size,
            depth=cubic_size,
            voxel_size=cubic_size / voxel_resolution,
            origin=[-cubic_size / 2.0, -cubic_size / 2.0, -cubic_size / 2.0]
        )
        self.cubic_size = cubic_size
        self.voxel_resolution = voxel_resolution
        # o3d.visualization.draw_geometries([voxel_grid], width=800, height=600, mesh_show_wireframe=True)

    def _normalize_geometry(self, geo: o3d.geometry.TriangleMesh):
        min_bound = geo.get_min_bound()
        max_bound = geo.get_max_bound()
        center = min_bound + (max_bound - min_bound) / 2.0
        vertices = np.array(geo.vertices)
        vertices -= center
        scale = np.linalg.norm(max_bound - min_bound) / 2.0
        geo.vertices = o3d.utility.Vector3dVector(vertices / scale)
        return geo

    def apply(self, mesh, w=300, h=300, out_geo=True):
        mesh.compute_vertex_normals()
        mesh = self._normalize_geometry(mesh)

        # setup visualizer to render depth maps
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=w, height=h, visible=False)
        vis.add_geometry(mesh)
        vis.get_render_option().mesh_show_back_face = True
        ctr = vis.get_view_control()
        # http://www.open3d.org/docs/release/python_api/open3d.visualization.ViewControl.html#open3d.visualization.ViewControl.convert_to_pinhole_camera_parameters
        camera_param = ctr.convert_to_pinhole_camera_parameters()

        center_pts = np.zeros((len(self.camera_pos.vertices), 3))
        for cid, xyz in enumerate(self.camera_pos.vertices):
            # Get new camera pose
            trans = get_extrinsic(xyz)
            camera_param.extrinsic = trans
            c = np.linalg.inv(trans) @ np.array([0, 0, 0, 1]).transpose()
            center_pts[cid, :] = c[:3]
            ctr.convert_from_pinhole_camera_parameters(camera_param)

            # Capture depth image and make a point cloud
            vis.poll_events()
            vis.update_renderer()
            depth = vis.capture_depth_float_buffer(False)

            # depth map carving method
            self.voxel_grid.carve_depth_map(o3d.geometry.Image(depth), camera_param)
            print("Carve view %03d/%03d" % (cid + 1, len(self.camera_pos.vertices)))
            if out_geo:
                o3d.io.write_voxel_grid(str(ROOT_PATH / "out" / "mesh_{:003}.ply".format(cid + 1)), self.voxel_grid)
        vis.destroy_window()


if __name__ == '__main__':
    mesh = get_armadillo_mesh()
    output_filename = str(ROOT_PATH / "assets" / "voxelized.ply")
    camera_path = str(ROOT_PATH / "assets" / "sphere.ply")
    cubic_size = 2.0
    voxel_resolution = 128.0

    voxel_carving = VoxelCarving(camera_path, cubic_size, voxel_resolution)
    voxel_carving.apply(mesh)
    o3d.visualization.draw_geometries([voxel_carving.voxel_grid], width=800, height=600)
