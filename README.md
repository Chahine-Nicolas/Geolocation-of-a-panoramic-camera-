## Geolocation of a panoramic camera by reference pairing

This repository presents the code mentioned in the JURSE 2023 paper: [Geolocation of a panoramic camera by reference pairing](https://hal.science/hal-04159503).
This project is the result of a Master 2 internship (also called "PFE", Projet de Fin d'Études) conducted between INSA Strasbourg and the INSIT Laboratory (HEIG-VD).

Presentation video: https://youtu.be/Nzk3pQtpLZE

The objective of this research is to propose a complementary solution to correct to pose estimation of a panoramic camera. 
Our approach pairs the horizon line detected within a fisheye image (observation) with a horizon line generated from a fictive camera position within a LiDAR tile from swissSURFACE3D.
This matching enables the creation of markers containing 2D image points (pixel coordinates) and corresponding 3D world points, which can then be used to estimate the camera pose using the collinearity equations.

## Installation

This implementation requires two distinct environments for compatibility reasons and must be executed within a Linux environment, since open3d.rendering.OffscreenRenderer is no longer supported on Windows.

### Main environment
```highlight
conda create -n Main_env python=3.10 -c conda-forge --strict-channel-priority
conda activate Main_env

conda install numpy=1.26 scipy matplotlib pillow shapely opencv sympy plotly laspy -c conda-forge
conda install open3d=0.18 -c conda-forge
conda install imutils numba gdal -c conda-forge

pip install dash dtw-python --no-deps
pip install ffmpeg-python
pip install haversine
```

Note: 
To import markers into Metashape during the final steps, you will need to install the Metashape [wheel file](https://www.agisoft.com/downloads/installer/).
You may also need an active license. However, all previous steps can be completed without Metashape.
```
pip install ..\metashape-2.3.0-cp39.cp310.cp311.cp312.cp313-none-win_amd64.whl 
```

### Torch environment

This environment is only used for sky detection within images.

```highlight
conda create -n torch python=3.11 -c conda-forge
conda activate torch

conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia
conda install matplotlib opencv -c conda-forge
conda install scikit-image open3d -c conda-forge
```

## Protocol

This project uses a GoPro Max 360 camera for geolocation experiments.
At least two videos are required:

- One video for IMU calibration, in which the camera undergoes rotations around all axes;
- A second video recorded around the area to geolocate.

The first step consists of projecting the .360 files into a sequence of front/back fisheye image pairs.

```highlight
python Gopro_2_fisheye.py -i \file_path\file.360 -o \file_path
```
<p align="center">
  <img src="https://github.com/Chahine-Nicolas/Geolocation-of-a-panoramic-camera-/blob/main/_assets/Frt_0000.png" width="300">
  <img src="https://github.com/Chahine-Nicolas/Geolocation-of-a-panoramic-camera-/blob/main/_assets/Bck_0000.png" width="300">
 
</p>








