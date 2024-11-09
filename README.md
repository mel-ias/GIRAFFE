# cpp_server_application

Server application of Open Water Levels. Runs all the heavyweight logic that has been outsourced from the mobile client

How to setup

1. create a new virtual python environemnt as prerequisite to run lightglue (ref) in the cloned project directory and follow the installation instruction from lightglue (https://github.com/cvg/LightGlue):
´´´
python -m venv .venv
.venv\Scripts\activate.bat
git clone https://github.com/cvg/LightGlue.git && cd LightGlue
python -m pip install .
´´´

2. setup the path to your lightglue installation in the config file of I2G


2. build I2G
Recommendation: using Buildtools for Visual Studio 2022 (if you do not consider Visual Studio 2022 IDE)

a. Download and Install Buildtools for Visual Studio 2022 from officile page: https://visualstudio.microsoft.com/de/downloads/#build-tools-for-visual-studio-2022
b. https://learn.microsoft.com/en-us/cpp/build/walkthrough-compiling-a-native-cpp-program-on-the-command-line?view=msvc-170&redirectedfrom=MSDN


3. build i2g to run by yourself using, for example, cmake.
Be sure that you provide all required packages compatbile with your platform and link to them
- opencv 4.10
- boost 1.73






