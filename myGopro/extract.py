import ffmpeg
import numpy as np
import subprocess
import os.path as op
from . import unstitch
from . import gps


def extractFromGopro(in_file, output_size=2720,
                     step_frame=10, use_index=True, no_images=False,
                     save_gps=None, step_dist=-1):

    probe = ffmpeg.probe(in_file)

    info = next(s for s in probe['streams'] if s['codec_type'] == 'video')
    width = int(info['width'])
    height = int(info['height'])

    fi = ffmpeg.input(in_file)

    pts_fr = None

    # map="0:3" for gps data
    stream = fi['3'].output("pipe:", format="rawvideo", codec="copy")
    pts_fr, out_data = gps.getFramesPositionsFromStream(stream)
    if save_gps is not None:
        with open(op.join(save_gps, "GPS.txt"), "w") as f:
            for k in pts_fr:
                p = pts_fr[k]
                f.write(f"{p.latitude:.8f} {p.longitude:.8f} {p.elevation:.3f}\n")
        with open(op.join(save_gps, 'CORI.txt'), "w") as f:
            for k in out_data['CORI']:
                if k != 'count':
                    f.write(f"{out_data['CORI'][k][0]} {out_data['CORI'][k][1]} {out_data['CORI'][k][2]} {out_data['CORI'][k][3]}\n")
        with open(op.join(save_gps, 'IORI.txt'), "w") as f:
            for k in out_data['IORI']:
                if k != 'count':
                    f.write(f"{out_data['IORI'][k][0]} {out_data['IORI'][k][1]} {out_data['IORI'][k][2]} {out_data['IORI'][k][3]}\n")
        with open(op.join(save_gps, 'GRAV.txt'), "w") as f:
            for k in out_data['GRAV']:
                if k != 'count':
                    f.write(f"{out_data['GRAV'][k][0]} {out_data['GRAV'][k][1]} {out_data['GRAV'][k][2]} \n")    
        with open(op.join(save_gps, 'MAGN.txt'), "w") as f:
            for k in out_data['MAGN']:
                if k != 'count':
                    f.write(f"{out_data['MAGN'][k][0]} {out_data['MAGN'][k][1]} {out_data['MAGN'][k][2]} \n")       
        with open(op.join(save_gps, 'GYRO.txt'), "w") as f:
            for k in out_data['GYRO']:
                if k != 'count':
                    f.write(f"{out_data['GYRO'][k][0]} {out_data['GYRO'][k][1]} {out_data['GYRO'][k][2]} \n")       
        with open(op.join(save_gps, 'ACCL.txt'), "w") as f:
            for k in out_data['ACCL']:
                if k != 'count':
                    f.write(f"{out_data['ACCL'][k][0]} {out_data['ACCL'][k][1]} {out_data['ACCL'][k][2]} \n")    

    if no_images:
        return

    nb_frames = len(pts_fr)
    s0 = fi['0'].trim(start_frame=0, end_frame=nb_frames)
    s5 = fi['5'].trim(start_frame=0, end_frame=nb_frames)

    args0 = s0.output('pipe:', format='rawvideo', pix_fmt='rgb24').compile()
    p0 = subprocess.Popen(args0, stdout=subprocess.PIPE)

    args5 = s5.output('pipe:', format='rawvideo', pix_fmt='rgb24').compile()
    p5 = subprocess.Popen(args5, stdout=subprocess.PIPE)

    idx = None
    if use_index:
        idx = unstitch.createFishEyeIndex(output_size)

    i = 0
    last_pt = gps.GPSPoint()
    while True:
        in_bytes0 = p0.stdout.read(width * height * 3)
        in_bytes5 = p5.stdout.read(width * height * 3)
        if not in_bytes0 or not in_bytes5:
            print("BREAK")
            break
        if not i % step_frame:
            f0 = np.frombuffer(in_bytes0, np.uint8).reshape([height, width, 3])
            f5 = np.frombuffer(in_bytes5, np.uint8).reshape([height, width, 3])
            pt = None
            if i in pts_fr:
                pt = pts_fr[i]
            if pt is not None and step_dist > 0:
                print(i, pt.distanceTo(last_pt))
                if pt.distanceTo(last_pt) > step_dist:
                    last_pt = pt
                else:
                    i += 1
                    continue
            out1, out2 = unstitch.unstitchGopro(f0, f5, output_size, index=idx)

            yield i, out1, out2, pt
        i += 1
