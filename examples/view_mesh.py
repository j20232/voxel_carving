import open3d as o3d
from pathlib import Path

ROOT_PATH = Path(".").resolve()


def get_bunny_mesh():
    bunny_path = ROOT_PATH / "assets" / "bunny.ply"
    mesh = 0
    return mesh


if __name__ == '__main__':
    mesh = get_bunny_mesh()
