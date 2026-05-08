import argparse
from pathlib import Path
import sys

import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import open3d as o3d
import open3d.visualization.rendering as rendering

from Prog import synchro as sc
from Prog import conversion as cv
from Prog import calib_magn as cal
from Prog import plot as plo
import reframe as ref
import lecture as lec

import gc

# ---- Constants ----
R = 6369000
a = 6378137
f = 1 / 298.2572236
b = a - a * f
e = np.sqrt(a**2 - b**2) / a


# ---------------------------------------------------------------------
# --------------------------- CORE FUNCTIONS --------------------------
# ---------------------------------------------------------------------

def load_data(link: Path, link_calib: Path):
    """Load all sensor data and calibration."""
    cori = lec.lecture_quat(link / "CORI.txt")
    euler = cv.quat_to_euler(cori)

    grav = lec.lecture_coord(link / "GRAV.txt")
    magn = lec.lecture_coord(link / "MAGN.txt")
    #gpslatlon = lec.lecture_coord(link / "GPS.txt")
    poslonlatav, poslonlatar = lec.lecture_pos(link / "pos.txt")

    magn_calib = lec.lecture_coord(link_calib / "MAGN.txt")
    center, normmagn = cal.calib_magn(magn_calib)

    return euler, grav, magn, poslonlatav, center, normmagn


def synchronize_data(poslonlatav, euler, grav, magn):
    """Synchronize signals."""
    pos_sync, euler_sync, grav_sync, magn_sync = sc.synchro_gps_fps(
        poslonlatav, euler, grav, magn
    )
    return pos_sync, euler_sync, grav_sync, magn_sync


def compute_orientation(i, pos_sync, grav_sync, heading, gpspt):
    """Compute camera orientation matrix."""
    # Projection
    ds = 2
    lon = pos_sync[i][1]
    lat = pos_sync[i][0]

    lon_proj = lon + (1 / (R * np.cos(lat * np.pi / 180))) * np.sin(heading[i] * np.pi / 180) * ds
    lat_proj = lat + (1 / R) * np.cos(heading[i] * np.pi / 180) * ds

    e_proj, n_proj = cv.sph2EN('plate_carree', lon_proj, lat_proj, R)
    e_proj -= gpspt[i][0]
    n_proj -= gpspt[i][1]

    magnx = np.array([
        e_proj / np.sqrt(e_proj**2 + n_proj**2),
        n_proj / np.sqrt(e_proj**2 + n_proj**2),
        0
    ])
    magnx /= norm(magnx)

    gravz = np.array([-grav_sync[i][0], -grav_sync[i][2], grav_sync[i][1]])
    gravz /= norm(gravz)

    cross = np.cross(gravz, magnx)
    cross /= norm(cross)

    R_mat = np.array([
        [magnx[0], cross[0], gravz[0]],
        [magnx[1], cross[1], gravz[1]],
        [magnx[2], cross[2], gravz[2]],
    ])

    center = R_mat[:, 0]
    up = R_mat[:, 2]

    return R_mat, center, up


def setup_renderer(mesh, width, height):
    """Initialize renderer."""
    material = rendering.MaterialRecord()
    render = rendering.OffscreenRenderer(width, height)
    render.scene.add_geometry('mesh', mesh, material)
    return render


def process_sequence(render, mesh, pos_sync, grav_sync, heading, out_dir, fov, width, height):
    """Main rendering loop."""
    gpspt = []
    pt = []

    for i in range(len(pos_sync)):
        # Convert GPS → EN
        e, n = cv.sph2EN('plate_carree', pos_sync[i][1], pos_sync[i][0], R)
        gpspt.append([e, n, pos_sync[i][2]])

        # Reframe
        xmn95, ymn95, zmn95 = pos_sync[i]
        xcam, ycam, zcam = ref.reframe(xmn95, ymn95, zmn95)
        pt.append([xcam, ycam, zcam])

        # Orientation
        R_mat, center, up = compute_orientation(i, pos_sync, grav_sync, heading, gpspt)

        S = np.array([xcam, ycam, zmn95])
        render.setup_camera(fov, S + center, S, up)

        # Render
        img = render.render_to_image()
        depth = render.render_to_depth_image(z_in_view_space=True)

        # Save
        out_img = out_dir / f"Frt_{i*10:04d}_mesh.png"
        out_depth = out_dir / f"Frt_{i*10:04d}_depth.png"

        o3d.io.write_image(str(out_img), img, 9)
        plt.imsave(str(out_depth), np.asarray(depth))

        print(f"[{i}] Saved image & depth at ", out_dir)
        
        del img
        del depth
        gc.collect()


# ---------------------------------------------------------------------
# ------------------------------ MAIN --------------------------------
# ---------------------------------------------------------------------

def main(args):
    base = Path(args.base_path)

    mesh_path = base / args.mesh
    mesh = o3d.io.read_triangle_mesh(str(mesh_path))

    link = base / args.sequence
    link_calib = base / args.calib_sequence

    out_dir = link
    out_dir.mkdir(parents=True, exist_ok=True)

    print("Loading data...")
    euler, grav, magn, poslonlatav, center, normmagn = load_data(link, link_calib)

    print("Synchronizing...")
    pos_sync, euler_sync, grav_sync, magn_sync = synchronize_data(
        poslonlatav, euler, grav, magn
    )

    print("Computing heading...")
    heading = plo.plot_magn(magn_sync, center, normmagn)

    print("Initializing renderer...")
    render = setup_renderer(mesh, args.width, args.height)

    print("Processing sequence...")
    process_sequence(render, mesh, pos_sync, grav_sync, heading, out_dir, args.fov, args.width, args.height)

    print("Done.")


# ---------------------------------------------------------------------
# -------------------------- ARGPARSE --------------------------------
# ---------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Render LiDAR + GoPro trajectory")

    parser.add_argument("--base-path", type=str,
                        default="/donnees",
                        help="Base data directory")

    parser.add_argument("--mesh", type=str,
                        default="Lidar/swisssurface3d_2019_2540-1181_2056_5728/tile.ply",
                        help="Mesh relative path")

    parser.add_argument("--sequence", type=str,
                        default="gopro/out/GS010191",
                        help="Sequence path")

    parser.add_argument("--calib-sequence", type=str,
                        default="gopro/out/GS010189",
                        help="Calibration sequence path")

    parser.add_argument("--width", type=int, default=2720)
    parser.add_argument("--height", type=int, default=2720)
    parser.add_argument("--fov", type=float, default=90.0)

    args = parser.parse_args()

    main(args)

