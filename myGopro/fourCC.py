import struct

maptype = {
    'c': ['c', 1],
    'L': ['L', 4],
    's': ['h', 2],
    'S': ['H', 2],
    'f': ['f', 4],
    'F': ['cccc', 4],  # 32-bit four character key -- FourCC
    'U': ['c', 1],
    'l': ['l', 4],
    'b': ['b', 1],
    'B': ['B', 1],
    'J': ['Q', 8]
    }

gpsf = {
    0: 'no lock (invalid GPS info)',
    2: 'lock 2D (ok)',
    3: 'lock 3D (ok)'
}


def map_type(type):
    ctype = chr(type)
    if ctype in maptype.keys():
        return maptype[ctype]
    return [ctype, -1]


def myManage(klvdata):
    # we need to check the REPEAT command.
    if not klvdata.rawdata:
        # empty data
        data = []
    else:
        data = []
        t, ts = map_type(klvdata.type)
        if ts < 0:
            if t not in ["#"]:
                print("warning, unknow type : ", t)
            return []
        si = klvdata.size//ts
        s = struct.Struct('>' + t * si)
        for r in range(klvdata.repeat):
            data_item = s.unpack_from(klvdata.rawdata[r*ts*si:(r+1)*ts*si])
            if si == 1:
                data_item = data_item[0]
            data.append(data_item)
        if klvdata.repeat == 1:
            return data_item
    return data
