from PIL import Image
import argparse
import sys
import os
import os.path as op
from scipy.spatial.transform import Rotation

from myGopro import extract
import sys
sys.path.append( r'C:\Users\chahi\Desktop\INSA\G5\PFE\prog')
import reframe as ref

def saveFile(out_dir, in_file,
             output_size, step_frame, step_dist, no_images,
             use_metashape, metashape_epsg):

    if use_metashape:
        try:
            import Metashape as ms
        except ImportError:
            print("Error : no module Metashape installed,",
                  "please install it before")
            use_metashape = False

    out_dir_loc = op.join(out_dir, op.splitext(op.basename(in_file))[0])
    if not op.isdir(out_dir_loc):
        os.mkdir(out_dir_loc)

    lstImgs = []
    lstGrps = []
    pts = {}
    # coris = {}
    with open(op.join(out_dir_loc, "pos.txt"), "w") as pos_file:
        for i, front, back, pt in extract.extractFromGopro(  # ori,grav,magn ?
                in_file,
                output_size=output_size,
                step_frame=step_frame,
                no_images=no_images,
                save_gps=out_dir_loc,
                step_dist=step_dist):
            print(i, pt)
            fn_frt = f"Frt_{i:04d}.png"
            fn_bck = f"Bck_{i:04d}.png"
            # save images
            Image.fromarray(front).save(op.join(out_dir_loc, fn_frt))
            Image.fromarray(back).save(op.join(out_dir_loc, fn_bck))
            # for metashape
            lstImgs.append(op.join(out_dir_loc, fn_frt))
            lstImgs.append(op.join(out_dir_loc, fn_bck))
            lstGrps.append(2)
            if pt is not None:
                pts[i] = pt
                # coris[i] = ori
                st = f"{pt.latitude:.8f} {pt.longitude:.8f} {pt.elevation:.3f}"
                pos_file.write(f"{fn_frt} {st}\n")
                pos_file.write(f"{fn_bck} {st}\n")

    if use_metashape:
        # Document courant
        doc = ms.Document()
        doc.save(op.join(out_dir_loc, "metashape.psx"))
        chunk = doc.addChunk()
        chunk.addPhotos(lstImgs, lstGrps, ms.MultiplaneLayout)
        chunk.crs = ms.CoordinateSystem(f"EPSG::{metashape_epsg}")
        chunk.camera_crs = ms.CoordinateSystem("EPSG::2056")
        chunk.euler_angles = ms.EulerAngles.EulerAnglesOPK
        for cam in chunk.cameras:
            i = int(cam.label.split("_")[1])
            if i in pts:
                pt = pts[i]
                xcam, ycam, zcam = ref.reframe(pt.latitude, pt.longitude, pt.elevation) 
                v = [xcam, ycam, pt.elevation]
                cam.reference.location = v
                # kpo = Rotation.from_quat(coris[i]).as_euler(
                #     'xyz', degrees=True)
                # cam.reference.rotation = ms.Vector(kpo[::-1])
        doc.save(op.join(out_dir_loc, "metashape.psx"))


########################################################################
def main(argv):

    parser = argparse.ArgumentParser(
        description="Extract panoramic images and gps data from Gopro max file (.360)")

    parser.add_argument(
        '-i', '--input_files',
        help='input videos .360 files (one or more)',
        type=str, nargs='*')
    parser.add_argument(
        '-o', '--output_dir',
        help='output dir, one subdirectory will be create for each input file',
        type=str, nargs='?')
    parser.add_argument(
        '-s', '--output_size',
        help='Output image size (default is 2720)',
        type=int, default=2720, nargs='?')
    parser.add_argument(
        '-t', '--step_frame',
        help='Take only one frame every X frames (default is 10)',
        type=int, default=10, nargs='?')
    parser.add_argument(
        '-d', '--step_dist',
        help='Take only one frame every X meters (default is -1)',
        type=float, default=-1, nargs='?')
    parser.add_argument(
        '-n', '--no_images',
        help='DOn\'t extract images, only GPS',
        action='store_true')
    parser.add_argument(
        '-m', '--metashape',
        help='Create Metashape project',
        action='store_true')
    parser.add_argument(
        '-e', '--metashape_epsg',
        help='Code EPSG for Metashape project projection (default is 2056)',
        type=int, default=2056, nargs='?')

    args = parser.parse_args()

    if len(sys.argv) <= 1:
        parser.print_help()
        exit()

    out_dir = ""
    if args.output_dir is not None:
        out_dir = args.output_dir

    if args.input_files is not None and len(args.input_files) > 0:
        for in_file in args.input_files:
            saveFile(out_dir, in_file, args.output_size,
                     args.step_frame, args.step_dist, args.no_images,
                     args.metashape, args.metashape_epsg)
    else:
        parser.print_help()
        exit()


########################################################################
# Main
########################################################################
if __name__ == "__main__":
    main(sys.argv)
