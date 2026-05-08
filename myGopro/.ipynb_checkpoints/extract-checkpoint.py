import ffmpeg
import numpy as np
import subprocess

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
    pts_fr, cori_frames, iori_frames = gps.getFramesPositionsFromStream(stream)
    if save_gps is not None:
        with open(save_gps, "w") as f:
            for k in pts_fr:
                p = pts_fr[k]
                f.write(f"{p.latitude:.8f} {p.longitude:.8f} {p.elevation:.3f}\n")

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
            ori = None
            if i in pts_fr:
                pt = pts_fr[i]
                ori = cori_frames[i]
            if pt is not None and step_dist > 0:
                print(i, pt.distanceTo(last_pt))
                if pt.distanceTo(last_pt) > step_dist:
                    last_pt = pt
                else:
                    i += 1
                    continue
            out1, out2 = unstitch.unstitchGopro(f0, f5, output_size, index=idx)

            yield i, out1, out2, pt, ori, f0, f5
        i += 1
