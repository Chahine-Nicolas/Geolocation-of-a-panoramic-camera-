from .klvdata import parseStream
from . import fourCC

import ffmpeg
from datetime import timedelta, date


class GPSPoint:
    def __init__(self, latitude=0.0, longitude=0.0,
                 elevation=0.0, time=date.today(), speed=0.0):
        self.latitude = latitude
        self.longitude = longitude
        self.elevation = elevation
        self.time = time
        self.speed = speed

    def __repr__(self):
        return f"GPSPoint({self.latitude},{self.longitude},{self.elevation},{self.time},{self.speed})"

    def __str__(self):
        return f"latitude:{self.latitude:.6f}, longitude:{self.longitude:.6f}, elevation:{self.elevation:.3f}, time:{self.time}, speed:{self.speed:.3f}"


def BuildGPSPoints(data, skip=False):
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
    SCAL = fourCC.XYZData(1.0, 1.0, 1.0)
    GPSU = None

    stats = {
        'ok': 0,
        'badfix': 0,
        'badfixskip': 0,
        'empty': 0,
        'GPS5_data': 0,
        'DISP': 0
    }
    sync_times = []
    sync_frames = []
    cori_frames = {}
    iori_frames = {}
    nb_frames_iori = 0
    nb_frames_cori = 0
    GPSFIX = 0  # no lock.
    for d in data:
        # print(f"{d.fourCC=}")
        if d.fourCC in stats:
            stats[d.fourCC] += 1
        else:
            stats[d.fourCC] = 0

        if d.fourCC == 'SCAL':
            SCAL = d.data

        # elif d.fourCC == 'SIUN':
        #    SIUN = d.data

        elif d.fourCC == 'CORI':
            for q in d.data:
                cori_frames[nb_frames_cori] = [float(x)/SCAL for x in q]
                nb_frames_cori+=1
        elif d.fourCC == 'IORI':
            for q in d.data:
                iori_frames[nb_frames_iori] = [float(x)/SCAL for x in q]
                nb_frames_iori+=1
        # elif d.fourCC == 'GRAV':
        #    print("GRAV",d.repeat,fourCC.map_type(d.type))
        #    print(SCAL,SIUN)
        #    print(d.data)
        # elif d.fourCC == 'MAGN':
        #    print("MAGN",d.repeat,fourCC.map_type(d.type))
        #    print(SCAL,SIUN)
        #    print(d.data)

        elif d.fourCC == 'GPSU':
            GPSU = d.data

        elif d.fourCC == 'GPSF':
            if d.data != GPSFIX:
                print("GPSFIX change to %s [%s]" % (d.data, fourCC.LabelGPSF.xlate[d.data]))
            GPSFIX = d.data

        elif d.fourCC == 'GPS5':
            sync_times.append(GPSU)
            sync_frames.append(stats['DISP'])
            GPSU_precise = GPSU
            # we have to use the REPEAT value.
            for item in d.data:
                if item.lon == item.lat == item.alt == 0:
                    print("Warning: Skipping empty point")
                    stats['empty'] += 1
                    continue

                if GPSFIX == 0:
                    stats['badfix'] += 1
                    if skip:
                        print("Warning: Skipping point due GPSFIX==0")
                        stats['badfixskip'] += 1
                        continue

                retdata = [float(x) / float(y) for x, y in zip(item._asdict().values(), list(SCAL))]

                gpsdata = fourCC.GPSData._make(retdata)
                p = GPSPoint(gpsdata.lat, gpsdata.lon, gpsdata.alt, GPSU_precise, gpsdata.speed)
                points.append(p)
                GPSU_precise += timedelta(milliseconds=55.5555)
                stats['ok'] += 1

    return points, stats, sync_times, sync_frames, cori_frames, iori_frames


def interp_pos(points, t):
    for i, pt in enumerate(points):
        if pt.time > t:
            break
    # pt.latitude,pt.longitude,pt.elevation,pt.time
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
    points, stats, sync_times, sync_frames, cori_frames, iori_frames = BuildGPSPoints(klvlist, skip=True)
    pts_fr = synchro(sync_times, sync_frames, points)
    return pts_fr, cori_frames, iori_frames


def getFramesPositionsFromFile(in_file):
    stream = ffmpeg.input(in_file)\
        .output("pipe:", format="rawvideo", map="0:3", codec="copy")
    return getFramesPositionsFromStream(stream)
