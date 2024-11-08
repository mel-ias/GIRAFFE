#pragma once

#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <string>


#include <windows.h>

#include "opencv2\opencv.hpp"

#include "json.hpp"
#include "LogfilePrinter.h"
#include "Utils.h"

#define _USE_MATH_DEFINES
#include <math.h>

#ifdef _WIN64
#include <direct.h>
#define GetCurrentDir _getcwd
#elif __linux__
#include <unistd.h>
#define GetCurrentDir getcwd
#endif

#endif


using nlohmann::json;

struct Recolored_Point_Cloud {

	cv::Point3d _point;
	cv::Vec3b _color;

	Recolored_Point_Cloud(cv::Point3d point, cv::Vec3b color) {
		_point = point;
		_color = color;
	}
};

class DataManager {

public:

	// C'tor
	// set default values for member variables
	DataManager(LogFile* logFile) {

		// --- from main ---
		_logfile = logFile;
		_path_file_point_cloud = "noPointCloud.bin"; // causes a "not a pw" error
		_path_working_directory = "noDir";
		_path_file_output_lightglue_matches = "noDir";
		_path_python_script_lightglue = "noFile";

		// --- from json ---
		_file_name_true_image = "noFileName";
		_file_name_image_points = "noFileName";		

		// parameters for IOP / about camera
		_principal_distance = 1.0;
		_pix_size = 0.00089;
		_view_angle_half_H = 30.0;
		_view_angle_half_V = 30.0;
		
		// parameters for EOP
		_X0_x = _X0_y = _X0_z = 0.0; // translation params: projC
		_shift_x = _shift_y = _shift_z = 0.0; // shift utm values for better handling
		_azimuth = _roll = _pitch = 0.0; //rotation params: Euler angles
		_rotM = new double[9]; // use only one instance for rotation matrix. for filling, just reference it
		_rotM[0] = 1.0; _rotM[3] = 0.0; _rotM[6] = 0.0;
		_rotM[1] = 0.0; _rotM[4] = 1.0; _rotM[7] = 0.0;
		_rotM[2] = 0.0; _rotM[5] = 0.0; _rotM[8] = 1.0;
		
		// thresholds / uncertainity values for point cloud projection
		_min_dist_to_X0 = 1.0;
		_max_dist_to_X0 = 200.0;
		_loc_acc_X0_z = 20.0;
		_loc_acc_X0_xy = 20.0;
		
		// others		
		_filter_matches_ransac_fisheye = 200.0; // run solvepnoransac to find good image-to-object matches before running spatial resection -> fisheye thresh (should be high cause OCVs solvepnp does not support fisheye cams)
		_filter_matches_ransac_pinhole = 8.0; // run solvepnoransac to find good image-to-object matches before running spatial resection -> pinhole thresh (should be small)

		// instantiate objects
		_frustum = new BoundingBox(logFile); // Instantiate frustum by BoundingBox to select point cloud part to be projected, provide pointer to logfile
		_coord_img = nullptr;

		_pts_synth_2D_double = new std::vector<cv::Point2d>;
		_pts_synth_3D_double = new std::vector<cv::Point3d>;
		_pts_color_RGB_int = new std::vector<cv::Scalar>;
		_point_cloud_recolored = new std::vector<Recolored_Point_Cloud>;
		_pts_image_points_2D = new std::vector<cv::Point2d>;	
	}

	// D'tor
	~DataManager() {
		if (_point_cloud_recolored != nullptr) { delete _point_cloud_recolored; }
		if (_pts_image_points_2D != nullptr) { delete _pts_image_points_2D; }
		if (_coord_img != nullptr) { delete _coord_img; }
		if (_pts_synth_2D_double != nullptr) { delete _pts_synth_2D_double; }
		if (_pts_synth_3D_double != nullptr) { delete _pts_synth_3D_double; }
		if (_pts_color_RGB_int != nullptr) { delete _pts_color_RGB_int; }
		if (_rotM != nullptr) { delete _rotM; }
		if (_frustum != nullptr) { delete _frustum; }	
	}



/**
 * @brief Reads and parses JSON configuration data from a specified file path, and updates internal parameters accordingly.
 *
 * This function reads a JSON file containing various parameters (e.g., file paths, camera settings, projection center,
 * orientation angles) required for processing 3D image projections and updates the internal parameters for subsequent
 * calculations. The function also handles default values and logs the results of each parameter's assignment.
 *
 * @param path_file_json Path to the JSON file containing configuration parameters.
 */
	void read_json_file(std::string path_file_json) {

		// Open the JSON file and parse its contents
		std::ifstream i(path_file_json);
		json j;
		i >> j;
		_logfile_json << "Read JSON file..." << std::endl;

		// Retrieve file name of the master (true) image
		if (j["file_name_true_image"] != nullptr) {
			_file_name_true_image = j.at("file_name_true_image").get<std::string>();
			_logfile_json << "Set 'file_name' of true image: " << _file_name_true_image << std::endl;
		}
		else {
			_logfile_json << "No value for 'file_name' in json." << std::endl;
		}

		// Retrieve file name of the image points to be referenced
		if (j["file_name_image_points"] != nullptr) {
			_file_name_image_points = j.at("file_name_image_points").get<std::string>();
			_logfile_json << "Set 'file_name' of image points file: " << _file_name_image_points << std::endl;
		}
		else {
			_logfile_json << "No value for 'file_name_image_points' in json." << std::endl;
		}

		// Read and set the focal length (in mm) if available
		if (j["focal_length_mm"] != nullptr) {
			_principal_distance = std::stod(j.at("focal_length_mm").get<std::string>());
			_logfile_json << "Set 'focal_length_mm' ck: " << _principal_distance << " [mm]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'focal_length_mm' in json." << std::endl;
		}

		// Retrieve the horizontal and vertical view angles
		if (j["view_angle_x"] != nullptr && j["view_angle_y"] != nullptr) {
			_view_angle_half_H = std::stod(j.at("view_angle_x").get<std::string>());
			_view_angle_half_V = std::stod(j.at("view_angle_y").get<std::string>());
			_logfile_json << "Set (full) opening angles 'view_angle_x'/'view_angle_y' (H/V): "
				<< _view_angle_half_H << "/" << _view_angle_half_V << " [°]" << std::endl;
		}
		else {
			_logfile_json << "No information about view angle horizontal and/or vertical of camera available" << std::endl;
		}

		// Set the initial projection center coordinates
		if (j["X0_x"] != nullptr && j["X0_y"] != nullptr && j["X0_z"] != nullptr) {
			_X0_x = std::stod(j.at("X0_x").get<std::string>());
			_X0_y = std::stod(j.at("X0_y").get<std::string>());
			_X0_z = std::stod(j.at("X0_z").get<std::string>());
			_logfile_json << "Set initial projection center 'X0 (x,y,z)': " << _X0_x << " [m], " << _X0_y << " [m], " << _X0_z << " [m]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'X0 (x,y,z)' in json." << std::endl;
		}

		// Set the orientation angles for azimuth, pitch, and roll if available
		if (j["azimuth"] != nullptr && j["pitch"] != nullptr && j["roll"] != nullptr) {
			_azimuth = std::stod(j.at("azimuth").get<std::string>());
			_pitch = std::stod(j.at("pitch").get<std::string>());
			_roll = std::stod(j.at("roll").get<std::string>());
			_logfile_json << "Set initial rotation angles (Euler, in deg): 'azimuth': " << _azimuth
				<< ", 'pitch': " << _pitch << ", 'roll': " << _roll << std::endl;
		}
		else {
			_logfile_json << "No value for 'azimuth', 'pitch', 'roll' in json." << std::endl;
		}

		// Set the pixel size in millimeters
		if (j["pixel_size_mm"] != nullptr) {
			_pix_size = std::stod(j.at("pixel_size_mm").get<std::string>());
			_logfile_json << "Set 'pixel_size_mm': " << _pix_size << " [mm]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'pixel_size_mm' in json, Set default: " << _pix_size << " [mm]" << std::endl;
		}

		// Set the location accuracy
		if (j["loc_accuracy"] != nullptr) {
			_loc_acc_X0_z = std::stod(j.at("loc_accuracy").get<std::string>());
			_loc_acc_X0_xy = _loc_acc_X0_z;
			_frustum->set_bb_offset_X0_xy(_loc_acc_X0_xy);
			_frustum->set_bb_offset_X0_z(_loc_acc_X0_z);
			_logfile_json << "Set 'loc_accuracy'. Noise position location (XY): " << _loc_acc_X0_xy
				<< " [m]" << std::endl;
			_logfile_json << "Set 'loc_accuracy'. Noise position height (Z): " << _loc_acc_X0_z << " [m]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'loc_accuracy' in json." << std::endl;
		}

		// Define the maximum distance to the projection center
		if (j["max_dist_to_X0"] != nullptr) {
			_max_dist_to_X0 = std::stod(j.at("max_dist_to_X0").get<std::string>());
			_logfile_json << "Set 'max_dist_to_X0': " << _max_dist_to_X0 << " [m]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'max_dist_to_X0' in json, set default: " << _max_dist_to_X0 << " [m]" << std::endl;
		}

		// Define the minimum distance from the projection center
		if (j["min_dist_to_X0"] != nullptr) {
			_min_dist_to_X0 = std::stod(j.at("min_dist_to_X0").get<std::string>());
			_logfile_json << "Set 'min_dist_to_X0': " << _min_dist_to_X0 << " [m]" << std::endl;
		}
		else {
			_logfile_json << "No value for 'min_dist_to_X0' in json, set default: " << _min_dist_to_X0 << " [m]" << std::endl;
		}

		// Load the true image from the specified file path
		fs::path path_file_trueImage = fs::path(path_file_json).parent_path() / _file_name_true_image;
		true_image = cv::imread(path_file_trueImage.string());
		_logfile_json << "Loaded true image from path: " << path_file_trueImage << ". Image size: " << true_image.size() << std::endl;

		// Load image points to be scaled
		if (_file_name_image_points != "noFileName") {
			fs::path path_file_image_points = fs::path(path_file_json).parent_path() / _file_name_image_points;
			std::ifstream inputStream(path_file_image_points);
			double x, y, z;
			char sep;
			while (inputStream >> x >> sep >> y >> sep >> z) {
				_pts_image_points_2D->emplace_back(x, y);
			}
			_logfile_json << "Loaded image points from path. Count: " << _pts_image_points_2D->size() << std::endl;
		}
		else {
			_logfile_json << "No image points to be scaled given." << std::endl;
		}

		// Adjust the view angles for frustum calculations
		_view_angle_half_H /= 2.0;
		_view_angle_half_V /= 2.0;
		_frustum->set_view_angles(_view_angle_half_H, _view_angle_half_V);
		_logfile_json << "Updated (half) view angles for frustum calculation (in deg): H: " << _view_angle_half_H << ", V: " << _view_angle_half_V << std::endl;

		// Apply coordinate shifts for efficiency
		_shift_x = _X0_x;
		_shift_y = _X0_y;
		_shift_z = _X0_z;
		_X0_x -= _shift_x;
		_X0_y -= _shift_y;
		_X0_z -= _shift_z;
		_logfile_json << "Applied shift to coordinates X0 for efficiency. shift_x: " << std::fixed << _shift_x << " [m], shift_y: " << _shift_y << " [m], shift_z: " << _shift_z << " [m]" << std::endl; //Use std::fixed floating-point notation for formatting
		
		_frustum->set_X0_Cam_World(_X0_x, _X0_y, _X0_z); //update frustum X0
		_frustum->calculate_rotation_matrix_rzxy(_azimuth, _roll, _pitch); //update frustum rotM
		
		// Precompute trigonometric values for rotation matrix based on azimuth, roll, and pitch angles
		double rad_azimuth = _azimuth * M_PI / 180.0;
		double rad_roll = _roll * M_PI / 180.0;
		double rad_pitch = _pitch * M_PI / 180.0;

		double Cz = cos(rad_azimuth);
		double Sz = sin(rad_azimuth);
		double Cy = cos(rad_roll);
		double Sy = sin(rad_roll);
		double Cx = cos(rad_pitch);
		double Sx = sin(rad_pitch);

		// Construct the rotation matrix _rotM (Rxyz) with rotations in the order of Rz (azimuth), Ry (roll), then Rx (pitch).
		// Rotational axes: x-axis (pitch), y-axis (roll), z-axis (azimuth).
		_rotM[0] = Cy * Cz;						_rotM[3] = -Cy * Sz;					_rotM[6] = Sy;
		_rotM[1] = Sx * Sy * Cz + Cx * Sz;		_rotM[4] = -Sx * Sy * Sz + Cx * Cz;	_rotM[7] = -Sx * Cy;
		_rotM[2] = -Cx * Sy * Cz + Sx * Sz;		_rotM[5] = Cx * Sy * Sz + Sx * Cz;	_rotM[8] = Cx * Cy;

		// Log the constructed rotation matrix for debugging
		_logfile->append(TAG + "RotM:");
		_logfile->append("\t\t" + std::to_string(_rotM[0]) + " " + std::to_string(_rotM[3]) + " " + std::to_string(_rotM[6]), 4);
		_logfile->append("\t\t" + std::to_string(_rotM[1]) + " " + std::to_string(_rotM[4]) + " " + std::to_string(_rotM[7]), 4);
		_logfile->append("\t\t" + std::to_string(_rotM[2]) + " " + std::to_string(_rotM[5]) + " " + std::to_string(_rotM[8]), 4);
		_logfile->append("");

		// Set maximum depth of the point cloud for projection within the frustum
		_frustum->set_frustum_depth(_max_dist_to_X0); 
		
		// Calculate the view frustum based on updated parameters
		_frustum->calculate_view_frustum();
		
		// Push log content to logfile
		std::string line, lineErr;
		_logfile->append(""); // Add an empty line before JSON content, if needed for separation
		while (std::getline(_logfile_json, line)) { // Read and log each line from _logfile_json with a "JSON:" prefix
			_logfile->append("JSON:\t\t" + line);
		}
	}


	/**
	 * @brief Runs LightGlue-based image matching on the provided true and synthetic images.
	 *
	 * This function saves the provided true and synthetic images to disk, generates an image list file,
	 * and executes a Python script to perform image matching using LightGlue. The matching results are
	 * saved in a designated output directory. The function checks for successful image and file storage
	 * before running the script to ensure data completeness.
	 *
	 * @param[in] in_true_image  The input image representing the "true" scene, as an OpenCV matrix (cv::Mat).
	 * @param[in] in_synth_image The input image representing the "synthetic" scene, as an OpenCV matrix (cv::Mat).
	 *
	 * @return True if the LightGlue matching script executes successfully, false otherwise.
	 */
	bool run_lightglue_image_matching(const cv::Mat& in_true_image, const cv::Mat& in_synth_image) {
		// Check if both input images are valid
		if (in_true_image.empty() || in_synth_image.empty()) {
			_logfile->append("Cannot run LightGlue image matching due to missing images.");
			return false;
		}
		else {
			_logfile->append("Running LightGlue image matching.");
		}

		// Define the directory path for input data and result storage
		fs::path path_directory_myData = get_path_working_directory() / "myData";
		fs::create_directory(path_directory_myData);  // Generate 'myData' directory if it doesn't exist

		// Define file paths for the image list and individual image files
		fs::path path_file_imagelist_to_match = path_directory_myData / "image_file_list.txt";
		fs::path path_file_trueImage_to_match = path_directory_myData / "true_image.jpg";
		fs::path path_file_synthImage_to_match = path_directory_myData / "synth_image.jpg";
		_path_file_output_lightglue_matches = path_directory_myData / "kpts.txt";

		// Save the images to the 'myData' directory
		cv::imwrite(path_file_trueImage_to_match.string(), in_true_image);
		cv::imwrite(path_file_synthImage_to_match.string(), in_synth_image);

		// Wait until the images are fully saved by checking their file sizes
		while (
			Utils::calculate_file_size(path_file_trueImage_to_match) == NULL ||
			Utils::calculate_file_size(path_file_synthImage_to_match) == NULL ||
			Utils::calculate_file_size(path_file_trueImage_to_match.string()) < 1 ||
			Utils::calculate_file_size(path_file_synthImage_to_match) < 1) {
			// Wait until both images have been completely written to disk
		}

		// Open an output file stream to write the image list file
		std::ofstream myFileImageList(path_file_imagelist_to_match);
		myFileImageList << path_file_trueImage_to_match << std::endl;
		myFileImageList << path_file_synthImage_to_match << std::endl;
		myFileImageList.close();

		// Ensure the image list file is saved before proceeding
		while (Utils::calculate_file_size(path_file_imagelist_to_match) == NULL) {
			// Wait until the image list file has been completely written to disk
		}

		// Prepare to execute the LightGlue Python script with arguments
		std::string python_executable = "python";  // Adjust this to "python3" or the path to your Python interpreter as needed

		// Construct the command to execute the Python script with appropriate arguments
		std::string command = python_executable + " " + this->get_path_python_script_lightglue().string() +
			" --left_image " + path_file_trueImage_to_match.string() +
			" --right_image " + path_file_synthImage_to_match.string() +
			" --output_dir " + path_directory_myData.string();

		// Execute the command to run the Python script
		int result = std::system(command.c_str());

		// Check if the Python script executed successfully
		if (result != 0) {
			std::cerr << "Error executing Python script." << std::endl;
			return false;
		}

		_logfile->append("Python-based LightGlue image matching executed successfully.");
		return true;
	}


	// -----------------
	// GETTERS / SETTERS 
	// -----------------

	// paths
	fs::path get_path_file_pointcloud() const { return _path_file_point_cloud; }
	fs::path get_path_working_directory() const { return _path_working_directory; }
	fs::path get_path_python_script_lightglue() const { return _path_python_script_lightglue; }
	fs::path get_path_lightglue_kpts() { return _path_file_output_lightglue_matches; }

	void set_path_working_directory(const fs::path& path_dir) {
		_path_working_directory = path_dir;
	}

	void set_path_file_pointcloud(const fs::path& path) {
		_path_file_point_cloud = path;
	}

	void set_path_python_script_lightglue(const fs::path& path_python_lightglue) {
		_path_python_script_lightglue = path_python_lightglue;
	}

	// set/get file name of image points file (only file name with extension, not path!)
	std::string get_file_name_image_points() { return _file_name_image_points; }
	void set_file_name_image_points(std::string name) { 
		_file_name_image_points = name; 
	}
	
	// logfile
	LogFile* get_logfile() { return _logfile; } 

	// image management 
	cv::Mat& get_synth_image() { return synth_image; } //get synth image
	cv::Mat& get_true_image() { return true_image; } //get true image
	CoordinateImage* get_coordinate_image() { return _coord_img; } // get coordinate image of projected point cloud
	std::string get_filename_true_image() { return _file_name_true_image; }	// get file name of true (master) image (set during read out of JSON meta data file)
	cv::Size get_size_true_image() { return true_image.size(); } // get image size of true image

	void set_synth_image(cv::Mat& synth_img) { synth_img.copyTo(synth_image); } //set synth image
	void set_true_image(cv::Mat& true_img) { true_img.copyTo(true_image); } //set true image 
	void set_coordinate_image(int column, int row) { _coord_img = new CoordinateImage(column, row); }	// set coordinate image of projected point cloud
	
	// point cloud management
	std::vector<cv::Point2d>* get_pts_synth_2D_double() { return _pts_synth_2D_double; }
	std::vector<cv::Point3d>* get_pts_synth_3D_double() { return _pts_synth_3D_double; }
	std::vector<cv::Scalar>* get_pts_color_RGB_int() { return _pts_color_RGB_int; }
	std::vector<Recolored_Point_Cloud>* get_point_cloud_recolored() { return _point_cloud_recolored; }
	std::vector<cv::Point2d>* get_image_points_2D_ptr() { return _pts_image_points_2D; } // pointer to image points to be referenced

	// coordinate shifts
	double get_shift_x() const { return _shift_x; }
	double get_shift_y() const { return _shift_y; }
	double get_shift_z() const { return _shift_z; }

	// frustum 
	BoundingBox* get_frustum() { return _frustum; }

	// camera orientation
	double& get_pixel_size() { return _pix_size; }
	double& get_principal_distance() { return _principal_distance; }
	double& get_min_dist_to_X0() { return _min_dist_to_X0; }
	cv::Point3d get_X0() { return cv::Point3d(_X0_x, _X0_y, _X0_z); }

	void set_X0_to_BBox(double x, double y, double z) {
		_X0_x = x;
		_X0_y = y;
		_X0_z = z;
		_frustum->set_X0_Cam_World(_X0_x, _X0_y, _X0_z);
	}

	// get rotation matrix
	double* get_rotM() { return _rotM; } 

	// set rotation matrix [row-major]
	void set_rotM(cv::Mat rotM) {
		_rotM[0] = rotM.at<double>(0, 0); _rotM[3] = rotM.at<double>(0, 1); _rotM[6] = rotM.at<double>(0, 2);
		_rotM[1] = rotM.at<double>(1, 0); _rotM[4] = rotM.at<double>(1, 1); _rotM[7] = rotM.at<double>(1, 2);
		_rotM[2] = rotM.at<double>(2, 0); _rotM[5] = rotM.at<double>(2, 1); _rotM[8] = rotM.at<double>(2, 2);
	}

	void set_filter_matches_ransac_fisheye(double val) {
		if (val > 0.0) {
			_filter_matches_ransac_fisheye = val;
		}
		else {
			_logfile->append("Invalid value for filter_matches_ransac_fisheye threshold. Must be val > 0. Restore default (200.0)");
			_filter_matches_ransac_fisheye = 200.0;
		}
	}
	double get_filter_matches_ransac_fisheye() const { return _filter_matches_ransac_fisheye; }

	void set_filter_matches_ransac_pinhole(double val) {
		if (val > 0.0) {
			_filter_matches_ransac_pinhole = val;
		}
		else {
			_logfile->append("Invalid value for filter_matches_ransac_pinhole threshold. Must be val > 0. Restore default (8.0)");
			_filter_matches_ransac_pinhole = 8.0;
		}
	}
	double get_filter_matches_ransac_pinhole() const { return _filter_matches_ransac_pinhole; }





private:

	// constants 
	// ---------
	const std::string TAG = "DataManager:\t";
	
	// paths to directories or files
	// -----------------------------
	fs::path _path_working_directory;
	fs::path _path_file_point_cloud;
	fs::path _path_file_output_lightglue_matches;
	fs::path _path_python_script_lightglue;

	std::string _file_name_true_image, _file_name_image_points;

	// logfile
	LogFile* _logfile;
	std::stringstream _logfile_json; 
	
	// image data
	cv::Mat true_image; 
	cv::Mat synth_image;

	// parameters
	double _pix_size, _loc_acc_X0_z, _loc_acc_X0_xy, _view_angle_half_H, _view_angle_half_V, _principal_distance; // for IOP
	double _X0_x, _X0_y, _X0_z; // for EOP
	double _shift_x, _shift_y, _shift_z; // enable shift of georeferenced point clouds (utm values very large numbers) 
	
	double _azimuth, _roll, _pitch; 
	double* _rotM;

	double _min_dist_to_X0; // for projection; check if point to be projected is to close to projection centre. use 1 m distance by default 
	double _max_dist_to_X0; // max depth of point cloud to be projected starting from projection cnetre. use 200 m distance by default

	double _filter_matches_ransac_fisheye;
	double _filter_matches_ransac_pinhole;

	// objects
	// -------
	BoundingBox* _frustum; 
	CoordinateImage* _coord_img;

	// vectors with data 
	// -----------------
	std::vector<cv::Point2d>* _pts_synth_2D_double; // for matching
	std::vector<cv::Point3d>* _pts_synth_3D_double; // for matching
	std::vector<cv::Scalar>* _pts_color_RGB_int; 	// for matching
	std::vector<cv::Point2d>* _pts_image_points_2D; // image points to be referenced
	std::vector<Recolored_Point_Cloud>* _point_cloud_recolored; // point cloud recolored from image points
};