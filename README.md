# GIRAFFE -  Geospatial Image Registration And reFErencing

![GIRAFFE - Logo](https://github.com/mel-ias/GIRAFFE/blob/I2G_publishing/GIRAFFE/docs/logo/banner.png?raw=true)

**GIRAFFE** is an open-source tool designed for precise registration and orientation of 2D images within 3D geometries, specifically point clouds. It enables image projection and alignment within the coordinate space of the point cloud, supporting workflows like visualization, georeferencing, and coloring.

## Features
- Image-to-Geometry Registration: Aligns images within a point cloud and links image data with geometric features.
- Image Rendering from Point Clouds: Generates synthetic views from point clouds to support image orientation.
- Feature Extraction and Matching: Detects and extracts image and geometry features for accurate spatial registration.
- Frustum Culling: Optimizes visibility and rendering by calculating a local frustum and transforming it into world coordinates.
- Automatic Calibration: Dynamically adjusts calibration parameters based on the distribution of matched points.

*Note: GIRAFFE includes the header-only library "JSON for Modern C++" (https://github.com/nlohmann/json). We would like to take the opportunity and thank the authors for providing this incredible useful library.*


## Installation

You can use the pre-built version given in directory VIG_Release_CV_410_x64 (Windows 10, 64 bit) or build it yourself using CMake, following the instructions below.


### Build Instructions

1. Install CMake
Download and install CMake (https://cmake.org/download/, minimum version 3.12; we tested with version 3.30 on Windows x64).
Note: Ensure CMake is added to your system PATH if not done automatically. To add it manually, you can use the following command in PowerShell or Command Prompt:
```set PATH="C:\Program Files\CMake\bin\";%PATH%```

2. Install Visual Studio 2022 MSVC 19.40 compiler
Download and install Visual Studio 2022 (the free Community Edition is sufficient) or, alternatively, the Build Tools for Visual Studio 2022 (https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022) to obtain the required MSVC compiler.
Note: We tested our compilation using MSVC 19.40, targeting Visual Studio 17 2022.

3. Clone the Repository
Clone this repository and switch to the project directory:
```
git clone https://github.com/mel-ias/GIRAFFE.git --branch I2G_publishing
cd GIRAFFE
```

4. Install Dependencies
Install the required dependency, OpenCV version 4.10.0 (https://opencv.org/releases/). Update the path to your OpenCV installation in the provided CMakeLists.txt file by setting OpenCV_DIR (e.g., ```set(OpenCV_DIR "C:/Libraries/OpenCV/opencv_4_10_win/build"```)).

5. Build GIRAFFE
Open PowerShell or Command Prompt in the project directory and build ```GIRAFFE.exe``` using CMake:

```
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

6. Copy the ```opencv4100_world.dll``` from your OpenCV installation to the directory of ```GIRAFFE.exe```

#### Integrate Lightglue for true-synthetic image matching
Create a new virtual python environemnt as prerequisite to run lightglue image matching in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue).  
We recommend to install Lightglue in a virtual Python 3.10. environment. Furthermore, be sure you run everything in command line (not powershell):

```
py -3.10 -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
```

*Note: If you do not want to use git, you can also download and unpack the lightglue repository. The user must switch to the unpacked repository (e.g. lightglue-main) and continue with ```python -m pip install .``` after the .venv has been activated.*

#### Copy initialisation file
```GIRAFFE.exe``` requires a few initialisation parameters that are preconfigured in the given ```init.json``` file that has to be copied to the directory of ```GIRAFFE.exe```. More details on the settings are given below.

## Usage
This example script demonstrates how to set up and execute the ```GIRAFFE.exe``` with sample arguments for processing a point cloud and image data.

### Example Batch Script Usage
Here is a sample batch script to configure and run GIRAFFE:

```
@echo off
:: Define paths for executable, virtual environment, and arguments
set "VIG_DIR=[...]\x64\VIG_Release_CV_410_x64"
set "VENV_DIR=%VIG_DIR%\.venv"
set "POINT_CLOUD_PATH=[...]\point_cloud.pw" :: PW format mandatory, use the provided PWConverter in the pre-built directory for conversion of a txt-saved point cloud given in X Y Z r g b format (space, comma or semicolon separated) by drag and drop the .txt point cloud file to the provided batch file xyzRGB_to_PW.bat
set "JSON_PATH=[...]\cam\cam_params.json"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py" :: link to the provided python file that executes lightglue-based matching, which is prepared in the program directory GIRAFFE/GIRAFFE. We recommend to copy the file to VIG_DIR.
set "PROJECT_NAME=my_project" :: specify project name

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause
```
#### Argument Details
- `-i`: Path to the point cloud file (in .pw format - see explanation above how to get the required PW-format).
- `-j`: Path to the JSON configuration file containing image or synthetic view data. (see explanation below)
- `-p`: Path to the Python script (e.g., match_pairs_lightglue.py) for feature matching.
- `-n`: Project name for the current run, used to organize output data.

## Additional Files
The **init.json** file configures GIRAFFE's initial parameters:
```
{
  "neighbour_distance": 2.5,
  "final_iteration_number": 3,
  "calc_IO": false,
  "ultra_wide_camera": false,
  "output_pointcloud": true
}
```

- neighbour_distance: Sets the maximum distance to consider neighbors.
- final_iteration_number: Defines the number of final alignment iterations.
- calc_IO: Enables or disables intrinsic calibration.
- ultra_wide_camera: Specifies if the camera is ultra-wide.
- output_pointcloud: Toggles output of the processed point cloud.

The **match_pairs_lightglue.py** python script handles feature matching using the LightGlue library. Key arguments (that are autmatically set and called by GIRAFFE) include:

```python
"--left_image: Path to the real image."
"--right_image: Path to the synthetic image."
"--output_dir: Directory for saving matched keypoints."
```
The script loads and processes images, extracts features, performs matching, and outputs matched keypoints to kpts.txt within the specified output_dir (match_pairs_lightglue).

#### JSON configuration file (-j)


```
### JSON Parameter Description

The following parameters are used in the JSON file:

- **`file_name_true_image`**: The name of the original image file (e.g., `"m220606150003016.jpg"`) that needs to be stored in the same directory as this json file
- **`azimuth`**: The azimuth angle in degrees. Indicates the camera's direction clockwise from north (e.g., `235`) in the reference system of the 3D point cloud data used for referencing the image measurements
- **`pitch`**: The pitch angle of the camera in degrees. Range: `0` (downward) to `180` (upward) (e.g., use `90` for close-range camera arrangements)
- **`roll`**: The roll angle of the camera in degrees. Describes the rotation around the camera's optical axis (e.g., `0` - usually used)
- **`X0_x`, `X0_y`, `X0_z`**: The cartesian coordinates (x, y, z) of the camera's projection centre in the reference system of the 3D point cloud (e.g., `2629114`, `1105319`, `2870`) - usually in metres
- **`loc_accuracy`**: The location accuracy in meters. Represents the uncertainty of the camera position (e.g., `0.5`) in the unit of the reference system; check the docs and Elias et al., 2019 for details on this parameter
- **`focal_length_mm`**: The focal length of the camera lens in millimeters (e.g., `5.4`)
- **`pixel_size_mm`**: The size of a single pixel of the camera sensor in millimeters (e.g., `0.003125`)
- **`view_angle_x`, `view_angle_y`**: The horizontal (`view_angle_x`) and vertical (`view_angle_y`) field of view of the camera in degrees (e.g., `90`, `90` for maximum field of view - recommended)
- **`file_name_image_points`**: A list of filenames containing the raw 2D image points to be referenced corresponding to the image coordinate system of true_image (e.g., `["time_lapse_sequence_01.txt", ..., "time_lapse_sequence_19.txt"]`). Note: the structure needs to be x,y,z (xy = image coordinates, z = 0)
- **`max_dist_to_X0`**: The maximum distance (in meters) from the camera origin (`X0`) to consider points in the view frustum (e.g., `3000`)
- **`min_dist_to_X0`**: The minimum distance (in meters) from the camera origin (`X0`) to consider points in the view frustum (e.g., `1.0`)
```  

### Documentation

To generate HTML documentation:

Install Doxygen.
Run:
bash

`
doxygen Doxyfile
`

The HTML documentation will be located in the docs folder.

### Contributing
Contributions are welcome! Please fork the repository and submit a pull request for review.


### Citation
If you use GIRAFFE in your work, please cite the underlying research papers as follows:  

#### BibTeX Entries  

```
@Article{HendrickxElias2024,
AUTHOR = {Hendrickx, H. and Elias, M. and Blanch, X. and Delaloye, R. and Eltner, A.},
TITLE = {AI-Based Tracking of Fast-Moving Alpine Landforms Using High Frequency Monoscopic Time-Lapse Imagery},
JOURNAL = {EGUsphere},
VOLUME = {2024},
YEAR = {2024},
PAGES = {1--20},
URL = {https://egusphere.copernicus.org/preprints/2024/egusphere-2024-2570/},
DOI = {10.5194/egusphere-2024-2570}
}


@article{Elias2023,
title = {Multi-modal image matching to colorize a SLAM based point cloud with arbitrary data from a thermal camera},
journal = {ISPRS Open Journal of Photogrammetry and Remote Sensing},
volume = {9},
pages = {100041},
year = {2023},
issn = {2667-3932},
doi = {https://doi.org/10.1016/j.ophoto.2023.100041},
url = {https://www.sciencedirect.com/science/article/pii/S2667393223000121},
author = {Melanie Elias and Alexandra Weitkamp and Anette Eltner},
keywords = {Thermal infrared (TIR) camera, Hand-held LiDAR, Urban mapping, Deep learning, Scene rendering},
}


@article{Elias2019,
author = {Elias, Melanie and Kehl, Christian and Schneider, Danilo},
title = {Photogrammetric water level determination using smartphone technology},
journal = {The Photogrammetric Record},
volume = {34},
number = {166},
pages = {198-223},
keywords = {exterior orientation, hydrology, image-to-geometry registration, outdoor application, sensor fusion, smartphone},
doi = {https://doi.org/10.1111/phor.12280},
url = {https://onlinelibrary.wiley.com/doi/abs/10.1111/phor.12280},
eprint = {https://onlinelibrary.wiley.com/doi/pdf/10.1111/phor.12280},
abstract = {Abstract Rapid technological progress has made mobile devices increasingly valuable for scientific research. This paper outlines a versatile camera-based water gauging method, implemented on smartphones, which is usable almost anywhere if 3D data is available at the targeted river section. After analysing smartphone images to detect the present water line, the image data is transferred into object space. Using the exterior orientation acquired by smartphone sensor fusion, a synthetic image originating from the 3D data is rendered that represents the local situation. Performing image-to-geometry registration using the true smartphone camera image and the rendered synthetic image, image parameters are refined by space resection. Moreover, the water line is transferred into object space by means of the underlying 3D information. The algorithm is implemented in the smartphone application “Open Water Levels”, which can be used on both high-end and low-cost devices. In a comprehensive investigation, the methodology is evaluated, demonstrating both its potential and remaining issues.},
year = {2019}
}





### License
This project is licensed under the MIT License. See the LICENSE file for details.