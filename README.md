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

1. Clone this repository:
```bash
git clone https://github.com/mel-ias/GIRAFFE.git --branch I2G_publishing
cd GIRAFFE
```
2. Dependencies: Install required dependencies as listed in the CMakeLists.txt file.
3. Build with CMake (tested with [MSVC Compiler using Buildtools für Visual Studio 2022](https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022 "MSVC Compiler using Buildtools für Visual Studio 2022")):

```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

#### Integrate Lightglue for true-synthetic image matching
Create a new virtual python environemnt as prerequisite to run lightglue image matching in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue).  We recommend to install Lightglue in a virtual Python 3.10. environment. Furthermore, be sure you run everything in command line (not powershell):

```bash
py -3.10 -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
```

#### Copy initialisation file
GIRAFFE.exe requires a few initialisation parameters that are preconfigured in the given init.json file that has to be copied to the directory of GIRAFFE.(.exe). More details on the settings are given below.

## Usage
This example script demonstrates how to set up and execute the GIRAFFE.exe with sample arguments for processing a point cloud and image data.

### Example Batch Script Usage
Here is a sample batch script to configure and run GIRAFFE:

```bash
@echo off
:: Define paths for executable, virtual environment, and arguments
set "VIG_DIR=[...]\x64\VIG_Release_CV_410_x64"
set "VENV_DIR=%VIG_DIR%\.venv"
set "POINT_CLOUD_PATH=[...]\point_cloud.pw" :: PW format mandatory
set "JSON_PATH=[...]\cam\cam_params.json"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py" :: link to the provided python file that executes lightglue-based matching
set "PROJECT_NAME=my_project" :: specify project name

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause
```
#### Argument Details
- `-i`: Path to the point cloud file (e.g., .pw format).
- `-j`: Path to the JSON configuration file containing image or synthetic view data. (see explanation below)
- `-p`: Path to the Python script (e.g., match_pairs_lightglue.py) for feature matching.
- `-n`: Project name for the current run, used to organize output data.

## Additional Files
The **init.json** file configures GIRAFFE's initial parameters:
bash:
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

TODO: ADD EXPLANATION HERE



points to be referenced: structure X,Y,Z - separated by commata, no exponential notation?!

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

### License
This project is licensed under the MIT License. See the LICENSE file for details.