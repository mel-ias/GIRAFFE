# GIRAFFE

How to setup

1. create a new virtual python environemnt as prerequisite to run lightglue (ref) in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue):
´´´
python -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
´´´

2. a) setup the path to your lightglue installation as argument and run prebuilt GIRAFFE.exe


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






