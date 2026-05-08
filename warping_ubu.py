import cv2
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d
import open3d.visualization.rendering as rendering

from scipy import ndimage
from numpy.linalg import norm
from dtw import dtw

import reframe as ref
import eq_colinea as col
import lecture as lec

import argparse

# --------------import argparse----------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------------

# BASE_DIR = "/mnt/c/Users/chahi/Desktop/INSA/G5/PFE"

# DATA_DIR = BASE_DIR + "/donnees"

# LIDAR_DIR = DATA_DIR + "/lidar/swisssurface3d_2019_2540-1181_2056_5728"

# GOPRO_DIR = DATA_DIR + "/gopro/out/GS010190"

# MESH_PATH = LIDAR_DIR + "/tile.ply"

# DEPTH_IMAGE = GOPRO_DIR + "/Frt_0240_mask.png"

# ROTATION_FILE = GOPRO_DIR + "/mat_rot.txt"

# POSITION_FILE = GOPRO_DIR + "/pos.txt"

WIDTH = 2720
HEIGHT = 2720
FOV = 90

# Camera parameters
cx, cy, f = 0.240779, -0.0230233, 1366.95
F = np.array([0, -1366, 0])

k1, k2, k3, k4 = (
    -0.170293,
    0.00611307,
    0.00174043,
    -0.000329237,
)

p1, p2, p3, p4 = (
    6.44318e-05,
    0.000241483,
    0,
    0,
)

b1, b2 = 0, 0

# ------------------------------------------------------------------
# FUNCTIONS
# ------------------------------------------------------------------


def load_binary_image(image_path):

    img = cv2.imread(str(image_path))

    if img is None:
        raise FileNotFoundError(f"Cannot load image: {image_path}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint8)

    binary = ndimage.binary_opening(
        gray,
        structure=np.ones((3, 3))
    ).astype(int)

    return img, binary


# ------------------------------------------------------------------


def find_horizon(image_path):

    print(f"Loading image: {image_path}")

    _, binary = load_binary_image(image_path)

    h, w = binary.shape

    iimg = []
    jimg = []

    for j in range(w):
        for i in range(h - 1):

            if binary[h - i - 1][j] == 1 and binary[h - i][j] != 1:

                if len(iimg) > j:
                    iimg[-1] = float(h - i)
                    jimg[-1] = float(j)
                else:
                    iimg.append(float(h - i))
                    jimg.append(float(j))

    return iimg, jimg


# ------------------------------------------------------------------


def create_horizon(mesh, center, up, camera_position):

    material = rendering.MaterialRecord()

    renderer = rendering.OffscreenRenderer(WIDTH, HEIGHT)

    renderer.scene.add_geometry("mesh", mesh, material)

    fovtype = (
        o3d.cpu.pybind.visualization.rendering.Camera.FovType.Horizontal
    )

    renderer.scene.camera.set_projection(
        FOV,
        1,
        0.1,
        1000,
        fovtype,
    )

    renderer.scene.camera.look_at(
        camera_position + center,
        camera_position,
        up,
    )

    depth = renderer.render_to_depth_image(z_in_view_space=True)

    depthmap = np.array(depth)

    mask = depthmap.copy()
    mask[mask != np.inf] = 255
    mask[mask == np.inf] = 0

    depthmap_open = ndimage.binary_opening(
        mask,
        structure=np.ones((3, 3)),
    ).astype(int)

    h, w = depthmap.shape

    imesh = []
    jmesh = []
    dmesh = []

    for j in range(w):
        for i in range(h - 1):

            if (
                depthmap_open[h - i - 2][j] == 0
                and depthmap_open[h - i - 1][j] != 0
            ):

                if j in jmesh:

                    idx = jmesh.index(j)

                    imesh[idx] = float(h - i - 1)
                    dmesh[idx] = depthmap[h - i - 1][j]

                else:

                    imesh.append(float(h - i - 1))
                    jmesh.append(float(j))
                    dmesh.append(depthmap[h - i - 1][j])

    return depthmap, imesh, jmesh, dmesh


# ------------------------------------------------------------------


def unproject(ihoriz, jhoriz, dhoriz, F, S, rotation):

    coord_img = [jhoriz, ihoriz]

    i_img, j_img = col.topixelCoord(
        coord_img,
        WIDTH,
        HEIGHT,
    )

    m = np.array([i_img, 0, j_img])

    M = dhoriz / norm(F) * rotation @ (m - F) + S

    return M


# ------------------------------------------------------------------


def dtw_horizon(image_path, mesh, S, R_temp):

    img = cv2.imread(str(image_path))

    if img is None:
        raise FileNotFoundError(image_path)

    img_b = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    up = np.array([
        R_temp[1][2],
        R_temp[0][2],
        R_temp[2][2],
    ])

    center = np.array([
        R_temp[0][1],
        R_temp[1][1],
        R_temp[2][1],
    ])

    # --------------------------------------------------------------
    # CREATE MESH HORIZON
    # --------------------------------------------------------------

    M = []

    depthmap, imesh, jmesh, dmesh = create_horizon(
        mesh,
        center,
        up,
        S,
    )

    for i in range(len(imesh)):

        M.append(
            unproject(
                imesh[i],
                jmesh[i],
                dmesh[i],
                F,
                S,
                R_temp,
            )
        )

    # --------------------------------------------------------------
    # IMAGE HORIZON
    # --------------------------------------------------------------

    iimg, jimg = find_horizon(image_path)

    # Sort points
    jmesh, imesh, dmesh = zip(*sorted(zip(jmesh, imesh, dmesh)))
    jimg, iimg = zip(*sorted(zip(jimg, iimg)))

    jmesh = list(jmesh)
    imesh = list(imesh)
    jimg = list(jimg)
    iimg = list(iimg)

    # --------------------------------------------------------------
    # VISUALIZATION
    # --------------------------------------------------------------

    plt.figure()
    plt.imshow(img_b, cmap='gray')
    plt.scatter(jimg, iimg, s=1, color="blue")
    plt.title('Image horizon ')

    plt.figure()
    plt.imshow(depthmap, cmap='gray')
    plt.scatter(jmesh, imesh, s=1, color="red")
    plt.title('Mesh horizon ')

    plt.figure()
    plt.scatter(jimg, iimg, s=1, color="blue")
    plt.scatter(jmesh, imesh, s=1, color="red")
    plt.axis("equal")
    plt.title("Horizon lines")

    # --------------------------------------------------------------
    # DTW ALIGNMENT
    # --------------------------------------------------------------

    query = np.array(imesh)
    reference = np.array(iimg)

    alignment = dtw(
        query,
        reference,
        keep_internals=True,
    )

    alignment.plot(type="twoway")

    print(f"DTW distance: {alignment.distance}")

    plt.show()

    return alignment


# ------------------------------------------------------------------
# MAIN
# ------------------------------------------------------------------


def main():

    parser = argparse.ArgumentParser(
        description="DTW horizon matching"
    )

    parser.add_argument(
        "--gopro_dir",
        type=str,
        default="/gopro/out/GS010190",
        help="Path to GoPro output directory"
    )

    parser.add_argument(
        "--mesh",
        type=str,
        default="/lidar/swisssurface3d_2019_2540-1181_2056_5728",
        help="Path to mesh .ply file"
    )

    parser.add_argument(
        "--image",
        type=str,
        default="Frt_0000",
        help="Image base name without extension"
    )

    parser.add_argument(
        "--fov",
        type=float,
        default=90,
        help="Camera FOV"
    )

    args = parser.parse_args()

    # ----------------------------------------------------------
    # BUILD PATHS
    # ----------------------------------------------------------

    depth_image = (
        args.gopro_dir
        + f"/{args.image}_mask.png"
    )

    rotation_file = (
        args.gopro_dir
        + "/mat_rot.txt"
    )

    position_file = (
        args.gopro_dir
        + "/pos.txt"
    )

    image_name = f"{args.image}.png"

    # ----------------------------------------------------------
    # CAMERA POSITION
    # ----------------------------------------------------------

    print("Loading camera position...")

    [xmn95, ymn95, zmn95], index = lec.lecture_coord_cam(
        image_name,
        position_file,
    )

    xcam, ycam, zcam = ref.reframe(
        xmn95,
        ymn95,
        zmn95,
    )

    # ----------------------------------------------------------
    # LOAD MESH
    # ----------------------------------------------------------

    print("Loading mesh...")

    mesh = o3d.io.read_triangle_mesh(args.mesh)

    # ----------------------------------------------------------
    # ROTATION
    # ----------------------------------------------------------

    print("Loading camera rotation...")

    R_camera = lec.lecture_matrix_cam(
        image_name,
        rotation_file,
    )

    R_agisoft = R_camera @ np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0],
    ])

    camera_position = np.array([
        xcam,
        ycam,
        zmn95,
    ])

    # ----------------------------------------------------------
    # RUN
    # ----------------------------------------------------------

    global FOV
    FOV = args.fov

    print("Running DTW horizon extraction...")

    dtw_horizon(
        depth_image,
        mesh,
        camera_position,
        R_agisoft,
    )

    print("Done")


# ------------------------------------------------------------------

if __name__ == "__main__":

    main()
    
# python warping_ubu.py --gopro_dir "/mnt/c/Users/chahi/Desktop/INSA/G5/PFE/donnees/gopro/out/GS010190" --mesh /mnt/c/Users/chahi/Desktop/INSA/G5/PFE/donnees/lidar/swisssurface3d_2019_2540-1181_2056_5728/tile.ply --image Frt_0000