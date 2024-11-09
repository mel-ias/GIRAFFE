# GIRAFFE


GIRAFFE is an open-source tool designed for precise registration and orientation of 2D images within 3D geometries, specifically point clouds. It enables image projection and alignment within the coordinate space of the point cloud, supporting workflows like visualization, georeferencing, and coloring.

## Features
- Image-to-Geometry Registration: Aligns images within a point cloud and links image data with geometric features.
- Image Rendering from Point Clouds: Generates synthetic views from point clouds to support image orientation.
- Feature Extraction and Matching: Detects and extracts image and geometry features for accurate spatial registration.
- Frustum Culling: Optimizes visibility and rendering by calculating a local frustum and transforming it into world coordinates.
- Automatic Calibration: Dynamically adjusts calibration parameters based on the distribution of matched points.


## Installation

1. Clone this repository:

bash:
`
git clone https://github.com/mel-ias/GIRAFFE.git --branch I2G_publishing
cd GIRAFFE
`

2. Dependencies: Install required dependencies as listed in the CMakeLists.txt file.
3. Build with CMake:

`
bash
Code kopieren
mkdir build
cd build
cmake ..
make
`

### Integrate Lightglue for true-synthetic image matching
4. create a new virtual python environemnt as prerequisite to run lightglue (ref) in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue):
`
python -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
`

5. copy "init.txt" to the program directory of GIRAFFE(.exe) 

### Integration of json.hpp
GIRAFFE includes the header-only library "JSON for Modern C++" (https://github.com/nlohmann/json). We would like to take the opportunity and thank the authors for providing this incredible useful library. 


## Usage

This example script demonstrates how to set up and execute the GIRAFFE.exe with sample arguments for processing a point cloud and image data.

### Example Batch Script Usage
Here is a sample batch script to configure and run GIRAFFE:

batch:

`
@echo off
:: Define paths for executable, virtual environment, and arguments
set "VIG_DIR=E:\PROMOTION\vs_workspace\I2G_publishing\cpp_server_application\x64\VIG_Release_CV_410_x64"
set "VENV_DIR=%VIG_DIR%\.venv"
set "POINT_CLOUD_PATH=E:\OTHER\PIPS\Data\PointCloud_Data\230703_Grabengufer_small2_5cm.pw"
set "JSON_PATH=E:\OTHER\PIPS\00_Review\I2G_Publishing_Test\cam4\synthImg.json"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py"
set "PROJECT_NAME=cam04_01"

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause
`

### Argument Details
- `-i`: Path to the point cloud file (e.g., .pw format).
- `-j`: Path to the JSON configuration file containing image or synthetic view data.
- `-p`: Path to the Python script (e.g., match_pairs_lightglue.py) for feature matching.
- `-n`: Project name for the current run, used to organize output data.

## Additional Files

init.json

The init.json file configures GIRAFFE’s initial parameters:

json
`
{
  "neighbour_distance": 2.5,
  "final_iteration_number": 3,
  "calc_IO": false,
  "ultra_wide_camera": false,
  "output_pointcloud": true
}
`

- neighbour_distance: Sets the maximum distance to consider neighbors.
- final_iteration_number: Defines the number of final alignment iterations.
- calc_IO: Enables or disables intrinsic calibration.
- ultra_wide_camera: Specifies if the camera is ultra-wide.
- output_pointcloud: Toggles output of the processed point cloud.

match_pairs_lightglue.py

match_pairs_lightglue.py handles feature matching using the LightGlue library. Key arguments include:
`
"--left_image: Path to the real image."
"--right_image: Path to the synthetic image."
"--output_dir: Directory for saving matched keypoints."
`

The script loads and processes images, extracts features, performs matching, and outputs matched keypoints to kpts.txt within the specified output_dir​(match_pairs_lightglue).


## Running GIRAFFE
To run the batch script:

Copy the script content into a .bat file.
Update paths as necessary for your setup.
Execute the batch file to start the GIRAFFE process with specified arguments.

Running GIRAFFE
To run GIRAFFE with the batch script:

Copy the script content into a .bat file.
Update paths as necessary.
Execute the batch file to start the GIRAFFE process

Documentation

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



How to setup



2. b) build GIRAFFE for your system using the prepared CMakeList.txt file (We recommend Buildtools for Visual Studio 2022 for this job (https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022)). Download OpenCV 4.10.0 for your system (using either the prebuilt version or built by your own using the OpenCV instructions) and link to your OpenCV installation in the CMakeList.txt file


3. GIRAFFE requires several to be runned. We recommend building a batch / shell script to run the tool without any efforts. An example for this is given below:

@echo off

:: Define the paths for the executable, virtual environment, and arguments
set "VIG_DIR=<path_to_the_dir_of_giraffe.exe" 
set "VENV_DIR=%VIG_DIR%\.venv" -> this requires that the .venv for lightglue is in the same directory as GIRAFFE.exe
set "POINT_CLOUD_PATH=<path_to_pw_file>"
set "JSON_PATH=<path_to_synth_file>"
set "PYTHON_SCRIPT_PATH=%VIG_DIR%\match_pairs_lightglue.py"
set "PROJECT_NAME=<project_name>"

:: Activate the virtual environment
call "%VENV_DIR%\Scripts\activate.bat"

:: Run GIRAFFE.exe with specified arguments
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"

:: Pause to keep the console open
pause






