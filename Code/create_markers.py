import os
import glob
import numpy as np
import reframe as ref
import lecture as lec
import warping as war
import convert_angle as cv
from Prog import synchro as sc
from Prog import calib_magn as cal
import argparse

# -------------------------------------------------------------------------
# CAMERA CONFIG
# -------------------------------------------------------------------------

def get_gopromax_intrinsics():
    """
    Retourne les paramètres intrinsèques caméra.
    """

    cx, cy, f = 0.240779, -0.0230233, 1366.95

    k1, k2, k3, k4 = -0.170293, 0.00611307, 0.00174043, -0.000329237
    b1, b2 = 0, 0
    p1, p2, p3, p4 = 6.44318e-05, 0.000241483, 0, 0

    w, h = 2720, 2720

    intrins = (
        cx, cy,
        k1, k2, k3, k4,
        p1, p2, p3, p4,
        b1, b2,
        w, h
    )

    return intrins


# -------------------------------------------------------------------------
# DATA LOADING
# -------------------------------------------------------------------------

def load_sensor_data(data_path):
    """
    Charge les données capteurs.
    """

    cori = lec.lecture_quat(os.path.join(data_path, 'CORI.txt'))
    euler = cv.quat_to_euler(cori)

    grav = lec.lecture_coord(os.path.join(data_path, 'GRAV.txt'))
    magn = lec.lecture_coord(os.path.join(data_path, 'MAGN.txt'))

    poslonlatav, poslonlatar = lec.lecture_pos(
        os.path.join(data_path, 'pos.txt')
    )

    return {
        "euler": euler,
        "grav": grav,
        "magn": magn,
        "gps": poslonlatav
    }


# -------------------------------------------------------------------------
# MAGNETOMETER CALIBRATION
# -------------------------------------------------------------------------

def compute_heading(data_path_calib, gps, euler, grav, magn):
    """
    Calibration magnétomètre + synchro.
    """

    magn_calib = lec.lecture_coord(
        os.path.join(data_path_calib, 'MAGN.txt')
    )

    center, normmagn = cal.calib_magn(magn_calib)

    pos_sync, euler_sync, grav_sync, magn_sync = (
        sc.synchro_gps_fps(gps, euler, grav, magn)
    )

    heading = cal.heading(magn_sync, center, normmagn)

    return heading


# -------------------------------------------------------------------------
# ROTATION
# -------------------------------------------------------------------------

def get_agisoft_rotation(cam_name, data_path):
    """
    Calcule la rotation Agisoft.
    """

    R_camera = lec.lecture_matrix_cam(
        cam_name,
        os.path.join(data_path, 'mat_rot.txt')
    )

    R_agisoft = R_camera @ np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0]
    ])

    return R_agisoft


# -------------------------------------------------------------------------
# IMAGE PROCESSING
# -------------------------------------------------------------------------

def process_image(
        img_path,
        data_path,
        heading,
        intrins,
        dtm_path,
        fov=90
):
    """
    Traite une image unique.
    """

    cam = os.path.basename(img_path)

    depth_img = cam[:-4] + "_mask.png"

    [xmn95, ymn95, zmn95], index = lec.lecture_coord_cam(
        cam,
        os.path.join(data_path, 'pos.txt')
    )

    xcam, ycam, zcam = ref.reframe(xmn95, ymn95, zmn95)

    S = np.array([xcam, ycam, zmn95])

    R_agisoft = get_agisoft_rotation(cam, data_path)

    frame_id = int(cam[4:-4]) // 10

    current_heading = heading[frame_id]

    print(f"{cam} | heading = {current_heading}")

    obs = war.warping(
        data_path,
        depth_img,
        fov,
        S,
        R_agisoft,
        intrins,
        current_heading,
        dtm_path
    )

    return obs


# -------------------------------------------------------------------------
# EXPORT
# -------------------------------------------------------------------------

def save_observations(observations, output_file):
    """
    Sauvegarde les observations.
    """

    with open(output_file, "w") as f:

        for i, obs_list in enumerate(observations):

            for obs in obs_list:

                f.write(
                    f"{i} "
                    f"{obs['X']} "
                    f"{obs['Y']} "
                    f"{obs['Z']} "
                    f"{obs['x']} "
                    f"{obs['y']}\n"
                )


# -------------------------------------------------------------------------
# MAIN PIPELINE
# -------------------------------------------------------------------------

def run_pipeline(
        data_path,
        calib_path,
        dtm_path,
        image_prefix="Frt",
        fov=90,
        output_name="mrkrtot.txt"
):
    """
    Pipeline principal.
    """

    intrins = get_gopromax_intrinsics()

    sensor_data = load_sensor_data(data_path)

    heading = compute_heading(
        calib_path,
        sensor_data["gps"],
        sensor_data["euler"],
        sensor_data["grav"],
        sensor_data["magn"]
    )

    observations = []

    image_pattern = os.path.join(
        data_path,
        f"{image_prefix}*0.png"
    )

    for img_path in glob.glob(image_pattern):

        obs = process_image(
            img_path,
            data_path,
            heading,
            intrins,
            dtm_path,
            fov=fov
        )

        observations.append(obs)

    output_file = os.path.join(data_path, output_name)

    save_observations(observations, output_file)

    print(f"Saved: {output_file}")


# -------------------------------------------------------------------------
# EXECUTION
# -------------------------------------------------------------------------

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description="Paths")
    parser.add_argument("--data_path", type=str,  default="data/", required=True, help="Gopro sequence path")
    parser.add_argument("--calib_path", type=str, default="data/", required=True, help="Gopro sequence calibration path")
    parser.add_argument("--dtm_path", type=str, default="data/raster.tif", required=True, help="raster.tif path")




    args = parser.parse_args()
    
    run_pipeline(
        data_path=args.data_path,
        calib_path=args.calib_path,
        dtm_path=args.dtm_path,
        image_prefix="Frt",
        fov=90
    )