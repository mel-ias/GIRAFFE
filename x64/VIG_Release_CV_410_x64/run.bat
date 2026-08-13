@echo off
set "VIG_DIR=D:\PIPS_GIRAFFE_GUI_v2\PIPS_GIRAFFE_GUI\GIRAFFE\x64\VIG_Release_CV_410_x64"
set "VENV_DIR=D:\PIPS_GIRAFFE_GUI_v2\PIPS_GIRAFFE_GUI\GIRAFFE\x64\VIG_Release_CV_410_x64\.venv"
set "POINT_CLOUD_PATH=D:/Hochebenkar/point_cloud/pointcloud_30072024_IGF_etrs89_utm32n.pw"
set "JSON_PATH=D:/Hochebenkar/giraffe_out/HEK_bilder_jun_sep_25\img_metadata.json"
set "PYTHON_SCRIPT_PATH=D:\PIPS_GIRAFFE_GUI_v2\PIPS_GIRAFFE_GUI\GIRAFFE\x64\VIG_Release_CV_410_x64\match_pairs_lightglue.py"
set "PROJECT_NAME=GIRAFFE_Results_20260810_134855"

set "PATH=%VENV_DIR%\Scripts;%PATH%"
"%VIG_DIR%\GIRAFFE.exe" -i "%POINT_CLOUD_PATH%" -j "%JSON_PATH%" -p "%PYTHON_SCRIPT_PATH%" -n "%PROJECT_NAME%"
