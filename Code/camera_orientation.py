import argparse
from pathlib import Path

import numpy as np
from numpy.linalg import norm

from Prog import synchro as sc
from Prog import calib_magn as cal

import convert_angle as cv
import lecture as lec


R_EARTH = 6369000
PROJ_DISTANCE = 2


# ============================================================
# IO
# ============================================================

def load_data(output_path, calib_path):
    """Load all sensor and calibration data."""

    output_path = Path(output_path)
    calib_path = Path(calib_path)

    cori = lec.lecture_quat(output_path / "CORI.txt")
    euler = cv.quat_to_euler(cori)

    grav = lec.lecture_coord(output_path / "GRAV.txt")
    magn = lec.lecture_coord(output_path / "MAGN.txt")

    poslonlatav, poslonlatar = lec.lecture_pos(output_path / "pos.txt")

    magn_calib = lec.lecture_coord(calib_path / "MAGN.txt")
    center, normmagn = cal.calib_magn(magn_calib)

    return {
        "euler": euler,
        "grav": grav,
        "magn": magn,
        "gps": poslonlatav,
        "center": center,
        "normmagn": normmagn,
    }


# ============================================================
# Math helpers
# ============================================================

def normalize(v):
    return v / norm(v)


def compute_mag_vector(pos, heading_deg):
    """Compute magnetic north vector."""

    lat, lon, _ = pos

    lon_proj = lon + (
        1 / (R_EARTH * np.cos(lat * np.pi / 180))
    ) * np.sin(np.radians(heading_deg)) * PROJ_DISTANCE

    lat_proj = lat + (
        1 / R_EARTH
    ) * np.cos(np.radians(heading_deg)) * PROJ_DISTANCE

    e, n = cv.sph2EN("plate_carree", lon, lat, R_EARTH)
    e2, n2 = cv.sph2EN("plate_carree", lon_proj, lat_proj, R_EARTH)

    vec = np.array([e2 - e, n2 - n, 0])

    return normalize(vec)


def compute_gravity_vector(grav):
    gravz = np.array([-grav[0], -grav[2], grav[1]])
    return normalize(gravz)


def compute_rotation_matrix(magny, gravz):
    estx = normalize(np.cross(magny, gravz))

    return np.array([
        [estx[0], gravz[0], -magny[0]],
        [estx[1], gravz[1], -magny[1]],
        [estx[2], gravz[2], -magny[2]],
    ])


# ============================================================
# Writers
# ============================================================

def write_euler(output_path, euler_sync):
    outfile = Path(output_path) / "CORI_2_euler.txt"

    with open(outfile, "w") as f:
        for angles in euler_sync:
            f.write(f"{angles[0]} {angles[1]} {angles[2]}\n")

    print("Wrote:", outfile)


def write_rotation_matrices(output_path, pos_sync, grav_sync, heading):
    outfile = Path(output_path) / "mat_rot.txt"

    with open(outfile, "w") as f:

        for i, pos in enumerate(pos_sync):

            magny = compute_mag_vector(pos, heading[i])
            gravz = compute_gravity_vector(grav_sync[i])

            R_agisoft = compute_rotation_matrix(magny, gravz)

            f.write(
                f"Frt_{i*10:04d}.png "
                f"{R_agisoft[0][0]} {R_agisoft[0][1]} {R_agisoft[0][2]} "
                f"{R_agisoft[1][0]} {R_agisoft[1][1]} {R_agisoft[1][2]} "
                f"{R_agisoft[2][0]} {R_agisoft[2][1]} {R_agisoft[2][2]}\n"
            )

    print("Wrote:", outfile)


# ============================================================
# Main pipeline
# ============================================================

def define_image_props(output_path, calib_path):

    data = load_data(output_path, calib_path)

    pos_sync, euler_sync, grav_sync, magn_sync = sc.synchro_gps_fps(
        data["gps"],
        data["euler"],
        data["grav"],
        data["magn"],
    )

    heading = cal.heading(
        magn_sync,
        data["center"],
        data["normmagn"],
    )

    write_euler(output_path, euler_sync)

    write_rotation_matrices(
        output_path,
        pos_sync,
        grav_sync,
        heading,
    )


# ============================================================
# CLI
# ============================================================

def parse_args():

    parser = argparse.ArgumentParser(
        description="Generate Agisoft rotation matrices from sensor data."
    )

    parser.add_argument(
        "--output",
        required=True,
        help="Path to output dataset directory",
    )

    parser.add_argument(
        "--calib",
        required=True,
        help="Path to calibration dataset directory",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    define_image_props(
        output_path=args.output,
        calib_path=args.calib,
    )


if __name__ == "__main__":
    main()