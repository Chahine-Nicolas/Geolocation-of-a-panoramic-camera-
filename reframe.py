import requests

def reframe(lat_wgs, long_wgs, alti_wgs):
    #print("reframe")
    # print(lat_wgs, lat_wgs, alti_wgs)
    transform_plani = "wgs84tolv95"
    transform_alti = "besseltolhn95"

    URL_plani = "http://geodesy.geo.admin.ch/reframe/" + transform_plani
    PARAMS_plani = {
        'easting' : long_wgs,
        'northing' : lat_wgs,
        'altitude' : alti_wgs,
        'format' : "json"}

    r_plani = requests.get(url = URL_plani, params = PARAMS_plani)
    data_mn95_bessel = r_plani.json()
    est_mn95 = data_mn95_bessel['easting']
    nord_mn95 = data_mn95_bessel['northing']
    alti_bessel = data_mn95_bessel['altitude']

    URL_alti = "http://geodesy.geo.admin.ch/reframe/" + transform_alti
    PARAMS_alti = {
        'easting' : est_mn95,
        'northing' : nord_mn95,
        'altitude' : alti_bessel,
        'format' : "json"}

    r_alti = requests.get(url = URL_alti, params = PARAMS_alti)
    data_mn95_ran95 = r_alti.json()
    alti_ran95 = data_mn95_ran95['altitude']

    return float(est_mn95), float(nord_mn95), float(alti_ran95)


lat = [46,46,43.19099999999]
lon = [6,38,29.871999999999]
lat_deg = lat[0]+lat[1]/60+lat[2]/3600
lon_deg = lon[0]+lon[1]/60+lon[2]/3600
xcam, ycam, zcam = reframe(lat_deg, lon_deg, 443.2)






