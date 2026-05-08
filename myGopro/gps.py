from .klvdata import parseStream
from . import fourCC

import ffmpeg
from datetime import timedelta, date
from haversine import haversine, Unit


class GPSPoint:
    def __init__(self, latitude=0.0, longitude=0.0,
                 elevation=0.0, time=date.today(), speed=0.0):
        self.latitude = latitude
        self.longitude = longitude
        self.elevation = elevation
        self.time = time
        self.speed = speed

    def distanceTo(self, other):
        pt1 = (self.latitude, self.longitude)
        pt2 = (other.latitude, other.longitude)
        return haversine(pt1, pt2, unit=Unit.METERS)

    def __repr__(self):
        return f"GPSPoint({self.latitude},{self.longitude},{self.elevation},{self.time},{self.speed})"

    def __str__(self):
        return f"latitude:{self.latitude:.6f}, longitude:{self.longitude:.6f}, elevation:{self.elevation:.3f}, time:{self.time}, speed:{self.speed:.3f}"


def extractGPMFData(data, skip=False):
    """
    Data comes UNSCALED so we have to do: Data / Scale.
    Do a finite state machine to process the labels.
    GET
     - SCAL     Scale value
     - GPSF     GPS Fix
     - GPSU     GPS Time
     - GPS5     GPS Data
    """

    points = []
    SCAL = 1.0
    GPSU = None

    stats = {
        'ok': 0,
        'badfix': 0,
        'badfixskip': 0,
        'empty': 0,
        'DISP': 0
    }
    sync_times = []
    sync_frames = []
    list_data_to_extract = ["CORI", "IORI", "GRAV", "MAGN", "GYRO", "ACCL"]
    out_data = {}
    for d in list_data_to_extract:
        out_data[d] = {}
        out_data[d]["count"] = 0

    GPSFIX = 0  # no lock.
    for d in data:

        if d.fourCC in stats:
            stats[d.fourCC] += 1
        else:
            stats[d.fourCC] = 0

        if d.fourCC == 'SCAL':
            SCAL = d.data

        if d.fourCC in list_data_to_extract:
            for q in d.scaledData(SCAL):
                co = out_data[d.fourCC]["count"]
                out_data[d.fourCC][co] = q
                out_data[d.fourCC]["count"] += 1

        elif d.fourCC == 'GPSU':
            GPSU = d.readTime()

        elif d.fourCC == 'GPSF':
            if d.data != GPSFIX:
                print(f"GPSFIX change to {d.data} {fourCC.gpsf[d.data]}")
            GPSFIX = d.data

        elif d.fourCC == 'GPS5':
            sync_times.append(GPSU)
            sync_frames.append(stats['DISP'])
            GPSU_precise = GPSU
            # we have to use the REPEAT value.
            for item in d.scaledData(SCAL):
                if item[0] == item[1] == item[2] == 0:
                    print("Warning: Skipping empty point")
                    stats['empty'] += 1
                    continue

                if GPSFIX == 0:
                    stats['badfix'] += 1
                    if skip:
                        # print("Warning: Skipping point due GPSFIX==0")
                        stats['badfixskip'] += 1
                        continue

                p = GPSPoint(item[0], item[1], item[2], GPSU_precise, item[3])
                points.append(p)
                GPSU_precise += timedelta(milliseconds=55.5555)
                stats['ok'] += 1
    for d in list_data_to_extract:
        print(d, out_data[d]["count"])

    return points, stats, sync_times, sync_frames, out_data


def interp_pos(points, t):
    if len(points) ==0:
        return GPSPoint(0, 0, 0, 0, 0)
    i=0
    for i, pt in enumerate(points):
        if pt.time > t:
            break
    # pt.latitude,pt.longitude,pt.elevation,pt.time
    if i == 0:
        p2 = points[i]
        lat = p2.latitude
        lon = p2.longitude
        ele = p2.elevation
        speed = p2.speed
    else:
        p1 = points[i-1]
        p2 = points[i]
        t1 = p1.time
        t2 = p2.time
        a = (t-t1).total_seconds()/(t2-t1).total_seconds()
        lat = p1.latitude + a*(p2.latitude - p1.latitude)
        lon = p1.longitude + a*(p2.longitude - p1.longitude)
        ele = p1.elevation + a*(p2.elevation - p1.elevation)
        speed = p1.speed + a*(p2.speed - p1.speed)
    return GPSPoint(lat, lon, ele, t, speed)


def synchro(sync_times, sync_frames, points):
    i = 0
    out = {}
    # Interpolation du temps:
    for fr in range(sync_frames[-1]):
        while sync_frames[i] < fr:
            i += 1
        if i == fr:
            out[fr] = interp_pos(points, sync_times[fr])
        else:
            t1 = sync_times[i-1]
            t2 = sync_times[i]
            a1 = sync_frames[i-1]
            a2 = sync_frames[i]
            dt = (t2-t1).total_seconds()*1000
            df = a2-a1
            t = t1 + timedelta(milliseconds=(fr-a1)/df*dt)
            out[fr] = interp_pos(points, t)
    return out


def getFramesPositionsFromStream(stream):
    data = stream.run(capture_stdout=True)[0]
    klvlist = parseStream(data)
    points, stats, sync_times, sync_frames, out_data = extractGPMFData(
        klvlist, skip=True)
    pts_fr = synchro(sync_times, sync_frames, points)
    return pts_fr, out_data


def getFramesPositionsFromFile(in_file):
    stream = ffmpeg.input(in_file)\
        .output("pipe:", format="rawvideo", map="0:3", codec="copy")
    return getFramesPositionsFromStream(stream)
