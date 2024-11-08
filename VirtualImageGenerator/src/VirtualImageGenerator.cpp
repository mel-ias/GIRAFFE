// VirtualImageGenerator.cpp : Defines the entry point for the console application.

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include "PerspectiveImage.h"
#include "Matching.h"
#include "json.hpp"
#include "DataManager.h"
#include <ctime>
#include "LogfilePrinter.h"

#include <iostream>
#include <string>
#include <filesystem>
#include <sstream>

#include "Utils.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <limits.h>
#endif


using json = nlohmann::json;
namespace fs = std::filesystem;

/**
 * @brief Tag constants for logging purposes.
 */
const::std::string NO_TAG = "";
const::std::string TAG = "Main:\t\t";
const::std::string ERROR_TAG = "Error:\t\t";

/**
 * @brief Pointers to main components in the application.
 */
DataManager* data_manager;     ///< Pointer to DataManager instance.
LogFile* log_printer;          ///< Pointer to LogFile instance for logging.
Matching* matching;            ///< Pointer to Matching instance.
PerspectiveImage* pim;         ///< Pointer to PerspectiveImage instance.

float _max_pixel_distance_synth_3D_derivation = 2.5f; ///< allowed pixel neighbor distance when grabbing a corresponding 3D point from synthetic image after image matching
int _final_iteration_number = 0;         ///< Tracks final iteration count.
bool _calc_IO = false;                   ///< Flag for input/output calculation.
bool _output_pointcloud = false;         ///< Flag to output the point cloud.
bool _have_fisheye = false;              ///< Flag indicating if fisheye lens is used.
double _balance_img_undistortion = 0.0;  ///< Balance parameter for undistortion.

/**
 * @brief Extracts the base name from a given path.
 *
 * @tparam T Type of the path (typically std::string or fs::path).
 * @param path Path from which to extract the base name.
 * @param delims Optional delimiters for extraction (default is "/\\").
 * @return The base name as a string.
 */
template<class T>
T base_name(T const& path, T const& delims = "/\\")
{
	// Verwende std::filesystem, um den Basisnamen zu extrahieren
	fs::path p(path);
	return p.filename().string();  // returns the base name (with extension)
}

/**
 * @brief Removes the extension from a filename.
 *
 * @tparam T Type of the filename (typically std::string or fs::path).
 * @param filename Filename from which to remove the extension.
 * @return The filename without extension as a string.
 */
template<class T>
T remove_extension(T const& filename)
{
	fs::path p(filename);
	return p.stem().string(); // returns the filename without extension
}

/**
 * @brief Terminates the program and frees allocated resources.
 *
 * @param error_message Message to log upon termination.
 * @return EXIT_FAILURE (typically -1) indicating unsuccessful program exit.
 */
int terminate_program(std::string error_message) {
	if (pim != nullptr)
		delete pim;
	if (data_manager != nullptr)
		delete data_manager;
	if (matching != nullptr)
		delete matching;

	log_printer->append(ERROR_TAG + error_message);

	if (log_printer != nullptr)
		delete log_printer;

	return (EXIT_FAILURE);
}

/**
 * @brief Retrieves the directory path of the executable.
 *
 * @return The path to the executable's directory as fs::path.
 * @throws std::runtime_error if the executable path cannot be determined.
 */
fs::path get_executable_path() {
	fs::path exe_path;

#ifdef _WIN32
	// Windows: Get the executable path using GetModuleFileName
	char buffer[MAX_PATH];
	if (GetModuleFileNameA(NULL, buffer, MAX_PATH) != 0) {
		exe_path = fs::path(buffer).parent_path();
	}
	else {
		std::cerr << "Error getting executable path." << std::endl;
	}
#else
	// Linux/Unix: Get the executable path using /proc/self/exe
	char buffer[PATH_MAX];
	ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
	if (len != -1) {
		buffer[len] = '\0'; // Null-terminate the string
		exe_path = fs::path(buffer).parent_path();
	}
	else {
		std::cerr << "Error getting executable path." << std::endl;
	}
#endif

	return exe_path;
}

 /**
  * @brief Reads initialization parameters from the init.json file located in the executable's directory.
  *        Sets relevant parameters in the DataManager and logs the configuration.
  *
  * This function checks if the init.json file exists, reads its contents, and updates several configuration
  * parameters used by the program. If required parameters are missing or incorrectly formatted, the program
  * terminates. Logs are created to document each parameter's value.
  *
  * @throws std::runtime_error if any required configuration parameter is missing or has an invalid value.
  */
static void read_init_file() {
	fs::path path_init = get_executable_path() / "init.json";  // Path to the init.json in executable directory

	std::cout << path_init.string() << std::endl;

	// Check if init file exists, otherwise quit
	if (!Utils::is_file(path_init)) {
		log_printer->append(TAG + "Could not find init.json. Please check!");
		return;
	}

	// Open init.json file
	std::ifstream f(path_init);
	try {
		// Parse the JSON configuration
		json j = json::parse(f);

		// Set filter_matches_ransac_pinhole
		if (j.contains("filter_matches_ransac_pinhole")) {
			float filter_matches_ransac_pinhole = j.at("filter_matches_ransac_pinhole").get<float>();
			data_manager->set_filter_matches_ransac_pinhole(filter_matches_ransac_pinhole);
			log_printer->append(TAG + " read init.txt, filter_matches_ransac_pinhole: " + std::to_string(filter_matches_ransac_pinhole));
		}
		else {
			terminate_program("read init.txt, no valid filter_matches_ransac_pinhole value found. Break.");
		}

		// Set filter_matches_ransac_fisheye
		if (j.contains("filter_matches_ransac_fisheye")) {
			float filter_matches_ransac_fisheye = j.at("filter_matches_ransac_fisheye").get<float>();
			data_manager->set_filter_matches_ransac_fisheye(filter_matches_ransac_fisheye);
			log_printer->append(TAG + " read init.txt, filter_matches_ransac_fisheye: " + std::to_string(filter_matches_ransac_fisheye));
		}
		else {
			terminate_program("read init.txt, no valid filter_matches_ransac_fisheye value found. Break.");
		}

		// Set neighbour distance
		if (j.contains("neighbour_distance")) {
			_max_pixel_distance_synth_3D_derivation = j.at("neighbour_distance").get<float>();
			log_printer->append(TAG + " read init.txt, neighbour_distance: " + std::to_string(_max_pixel_distance_synth_3D_derivation));
		}
		else {
			terminate_program("read init.txt, no valid neighbour distance value found. Break.");
		}

		// Set final iteration number
		if (j.contains("final_iteration_number")) {
			_final_iteration_number = j.at("final_iteration_number").get<int>();
			log_printer->append(TAG + " read init.txt, final_iteration_number: " + std::to_string(_final_iteration_number));
		}
		else {
			terminate_program("read init.txt, no valid final_iteration number value found. Break.");
		}

		// Set calc_IO flag
		if (j.contains("calc_IO")) {
			_calc_IO = j.at("calc_IO").get<bool>();
			log_printer->append(TAG + " read init.txt, will " + std::string(_calc_IO ? "calc_IO" : "NOT calc_IO"));
		}

		// Set fisheye flag
		if (j.contains("fisheye")) {
			_have_fisheye = j.at("fisheye").get<bool>();
			log_printer->append(TAG + " read init.txt, using " + std::string(_have_fisheye ? "fisheye camera model" : "central perspective camera model") + " for final epoch if refine_iop is true");
		}

		// Set balance undistortion parameter
		if (j.contains("balance_undist")) {
			_balance_img_undistortion = j.at("balance_undist").get<double>();
			log_printer->append(TAG + " read init.txt, balance_undist: " + std::to_string(_balance_img_undistortion));
		}
		else {
			terminate_program("read init.txt, no valid balance_undist value found. Break.");
		}

		// Set output_pointcloud flag
		if (j.contains("output_pointcloud")) {
			_output_pointcloud = j.at("output_pointcloud").get<bool>();
			log_printer->append(TAG + " read init.txt, will " + std::string(_output_pointcloud ? "save" : "NOT save") + " true-image-colored point cloud to disk");
		}
	}
	catch (const json::parse_error& e) {
		// Handle JSON parsing error
		std::cerr << "Parse error: " << e.what() << std::endl;
	}
}
		

int main(int argc, char** argv) {

	// Initialize log file printer and data manager
	log_printer = new LogFile();
	data_manager = new DataManager(log_printer);  // Initialize data manager for data storage and management
	log_printer->append(TAG + "initialized 'logFilePrinter' and 'dataManager'");

	// Check if necessary arguments are valid; otherwise, terminate before doing anything
	if (argc < 2) {
		terminate_program("Necessary arguments not provided. Please ensure -i, -j, -n, and -p are given.");
	}

	log_printer->append(TAG + "START VIRTUAL-IMAGE-GENERATOR");

	// Parse command-line arguments
	fs::path path_file_pointcloudPW, path_file_jsonTxt, path_python_script_lightglue;
	std::string working_directory_name;
	for (int i = 1; i < argc; ++i) {
		char check = argv[i][1];
		switch (check) {
		case 'i': {
			// Point cloud file
			path_file_pointcloudPW = fs::path(argv[++i]);
			if (!fs::exists(path_file_pointcloudPW)) {
				std::cerr << "Error: Point cloud file does not exist: " << path_file_pointcloudPW << std::endl;
				return -1;
			}
			log_printer->append(TAG + "Set path to point cloud (PW-file): " + path_file_pointcloudPW.string());
			data_manager->set_path_file_pointcloud(path_file_pointcloudPW);  // Set path for input point cloud
			break;
		}

		case 'j': {
			// JSON file for configuration
			path_file_jsonTxt = fs::path(argv[++i]);
			if (!fs::exists(path_file_jsonTxt)) {
				std::cerr << "Error: JSON file does not exist: " << path_file_jsonTxt << std::endl;
				return -1;
			}
			log_printer->append(TAG + "Set path to JSON configuration (TXT-file): " + path_file_jsonTxt.string());
			data_manager->read_json_file(path_file_jsonTxt.string());  // Read JSON file
			log_printer->append(TAG + "Read JSON file successfully");
			break;
		}

		case 'p': {
			// Python script path for LightGlue processing
			path_python_script_lightglue = fs::path(argv[++i]);
			if (!fs::exists(path_python_script_lightglue)) {
				std::cerr << "Error: Python script file does not exist: " << path_python_script_lightglue << std::endl;
				return -1;
			}
			log_printer->append(TAG + "Set path to Python script for LightGlue: " + path_python_script_lightglue.string());
			data_manager->set_path_python_script_lightglue(path_python_script_lightglue);
			break;
		}

		case 'n': {
			// Working directory name
			working_directory_name = std::string(argv[++i]);
			if (working_directory_name.empty()) {
				std::cerr << "Error: No working directory name provided." << std::endl;
				return -1;
			}
			log_printer->append(TAG + "Set working directory name to: " + working_directory_name);
			break;
		}

		default: {
			std::cerr << "Unknown argument: " << argv[i] << std::endl;
			return -1;
		}
		}
	}

	// Set up result and working directories
	fs::path path_dir_result = fs::current_path() / "Results";  // Results folder in the current working directory
	fs::path path = path_dir_result / working_directory_name;

	// Check if "Results" directory exists; if not, create it
	if (!fs::exists(path_dir_result)) {
		if (!fs::create_directory(path_dir_result)) {
			std::cerr << "Error: Could not create 'Results' directory!" << std::endl;
			return 1;
		}
	}

	// Check if the specified working directory already exists; if so, add a counter suffix
	int dirCounter = 0;
	while (fs::exists(path) && dirCounter < 10) {
		dirCounter++;
		path = path_dir_result / (working_directory_name + "(" + std::to_string(dirCounter) + ")");
	}

	// Create the final working directory if it does not exist
	if (!fs::create_directories(path)) {
		std::cerr << "Error: Could not create output directory!" << std::endl;
		return 1;
	}

	// Set paths in the data manager
	data_manager->set_path_working_directory(path);
	log_printer->append(TAG + "Working directory set to: " + path.string());


	// ------------------
	// READ INIT.TXT FILE
	// ------------------
	read_init_file();  // Load additional settings from init.json


	// ----------------
	// START PROCESSING
	// ----------------

	cv::Mat camera_matrix, dist_coeffs, tVecObj, rMatObj, stdDevCam_In, stdDevObj_Ext;
	float repro_error;
	for (int iteration = 0; iteration <= _final_iteration_number; ++iteration) {

		log_printer->append(TAG + "------------------ ITERATION " + std::to_string(iteration) + "/" + std::to_string(_final_iteration_number) + "---------------------");

		//init FLAGS
		Matching::Flags_resec flags_matching = Matching::Flags_resec::CALC_EO_IO; //init

		
		// first iteration, fix IO + distC and optimise EO only
		if (_calc_IO && iteration > 1) { // flag for IOP refinement set and not first iteration, refine IO
			flags_matching = Matching::CALC_EO_IO;
			log_printer->append(TAG + "Matching Flags: " + "CALC_EO_IO" + " (IOP refinement allowed) ");
		}
		else {
			flags_matching = Matching::CALC_EO;  //refine IO if flag is false (IO refinement denied) otherwise IO fixed
			log_printer->append(TAG + "Matching Flags: " + "CALC_EO" + " (IOP refinement denied) ");
		}
		log_printer->append(TAG + "set flags_matching variable. checked extr parameters ");



		// a) generate virtual image (central perspective), store in dataManager 
		pim = new PerspectiveImage(data_manager);
		pim->generateImage();


		// b) prepare true/synth image matching
		cv::Mat true_img_4matching, synth_img_4matching;
		data_manager->get_true_image().copyTo(true_img_4matching);
		data_manager->get_synth_image().copyTo(synth_img_4matching);

		// init IO
		// prepare camera matrix + dist_coeffs using approximations from init file (use of pre-calib cams removed because of diverse camera models; instead use undistorted images)
		double focal_length_px = data_manager->get_principal_distance() / data_manager->get_pixel_size();
		cv::Point2d center = cv::Point2d(data_manager->get_true_image().cols / 2, data_manager->get_true_image().rows / 2); // calc image center for initial principle point				
		camera_matrix = cv::Mat(3, 3, CV_64FC1);
		camera_matrix = (cv::Mat_<double>(3, 3) << focal_length_px, 0, center.x, 0, focal_length_px, center.y, 0, 0, 1);

		// dist_coeffs: fisheye model uses less coeffs than central perspective
		dist_coeffs = (_have_fisheye) ? cv::Mat::zeros(4, 1, CV_64FC1) : cv::Mat::zeros(5, 1, CV_64FC1);


		// Initialize Exterior Orientation (EO) for solvePnPRansac
		// Note: solvePnPRansac requires object pose in camera coordinates. Convert before use.
		
		// Convert camera pose in object coordinate system to object pose in camera coordinate system
		tVecObj = cv::Mat(); // Translation vector
		rMatObj = cv::Mat(3, 3, CV_64FC1); // Rotation matrix

		// Set translation vector from projection center coordinates
		tVecObj.push_back(data_manager->get_X0().x);
		tVecObj.push_back(data_manager->get_X0().y);
		tVecObj.push_back(data_manager->get_X0().z);

		// Set rotation matrix from data_manager rotation matrix pointer
		double* rotM_ptr = data_manager->get_rotM(); // Pointer to 9-element rotation matrix (row-major order)
		rMatObj.at<double>(0, 0) = rotM_ptr[0]; rMatObj.at<double>(0, 1) = rotM_ptr[3]; rMatObj.at<double>(0, 2) = rotM_ptr[6];
		rMatObj.at<double>(1, 0) = rotM_ptr[1]; rMatObj.at<double>(1, 1) = rotM_ptr[4]; rMatObj.at<double>(1, 2) = rotM_ptr[7];
		rMatObj.at<double>(2, 0) = rotM_ptr[2]; rMatObj.at<double>(2, 1) = rotM_ptr[5]; rMatObj.at<double>(2, 2) = rotM_ptr[8];

		// Convert to view mode required by solvePnPRansac
		cv::transpose(rMatObj, rMatObj); // Transpose rotation matrix
		tVecObj = -rMatObj * tVecObj; // Calculate new translation vector

		// Initialize matching process and start image-to-geometry intersection
		matching = new Matching();
		matching->init(data_manager);

		// Perform spatial resection if intrinsic and extrinsic parameters are not fixed
		if (flags_matching != Matching::FIXED_EO_IO) {

			// Initialize vectors for storing matching points between synthetic and real images
			std::vector<cv::Point3d> matched_object_points;
			std::vector<cv::Point2d> matched_image_points_real, matched_image_points_synth;

			// Log start of feature detection and matching process
			log_printer->append(TAG +
				"-----------------------------------------" + "\n" +
				"START FEATURE DETECTION / MATCHING : LIGHTGLUE" + "\n" +
				"-----------------------------------------");

			// Run LightGlue image matching between true and synthetic images
			if (data_manager->run_lightglue_image_matching(true_img_4matching, synth_img_4matching)) {

				// Define output path for LightGlue keypoints
				std::string path_matching_out = data_manager->get_path_lightglue_kpts().string();

				// Log successful matching and start reading inliers that passed the fundamental test
				log_printer->append(TAG + "matching successful, read inliers passing fundamental test...");

				// Wait until the matching output file is saved and has valid content
				while (Utils::calculate_file_size(path_matching_out) == NULL || Utils::calculate_file_size(path_matching_out) < 1) {}

				// Run post-processing on inlier data from VisualSFM
				// Calculate camera parameters for transformation into object space
				matching->loadMatches(
					path_matching_out,
					data_manager->get_true_image(),
					data_manager->get_synth_image(),
					*data_manager->get_image_points_2D_ptr(),
					*data_manager->get_pts_synth_2D_double(),
					*data_manager->get_pts_synth_3D_double(),
					matched_object_points,
					matched_image_points_real,
					matched_image_points_synth,
					//scale_true_img,
					//scale_synth_img,
					_max_pixel_distance_synth_3D_derivation);
				
				// Perform spatial resection to determine camera and exterior orientation, check if values are valid
				repro_error = matching->space_resection(
					matched_object_points,
					matched_image_points_real,
					data_manager->get_true_image(),
					data_manager->get_pixel_size(),
					camera_matrix,
					dist_coeffs,
					rMatObj,
					tVecObj,
					stdDevCam_In,
					stdDevObj_Ext,
					flags_matching,
					_have_fisheye);

				// If Interior Orientation Parameters (IOP) are calculated, undistort the image to align synthetic and real images for better matching
				if (flags_matching == Matching::CALC_EO_IO) { // Before last iteration, undistort and use the undistorted image in the final iteration
					cv::Mat undist_true_image;
					cv::Mat camera_matrix_new = camera_matrix.clone();
					if (_have_fisheye) {
						// For fisheye lens, estimate a new camera matrix and undistort using fisheye functions
						cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
						cv::Mat map1;
						cv::Mat map2;
						cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, data_manager->get_size_true_image(), E, camera_matrix_new, _balance_img_undistortion);
						cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, E, camera_matrix_new, data_manager->get_size_true_image(), CV_16SC2, map1, map2);
						cv::remap(data_manager->get_true_image(), undist_true_image, map1, map2, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);
					}
					else {
						// For non-fisheye lens, use the optimal new camera matrix and standard undistort method
						camera_matrix_new = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, data_manager->get_size_true_image(), _balance_img_undistortion);
						cv::undistort(data_manager->get_true_image(), undist_true_image, camera_matrix, dist_coeffs);
					}

					// Save the undistorted image and update data manager with new image and camera matrix
					fs::path out_path_undist_img = data_manager->get_path_working_directory() / ("undist_test_" + std::to_string(iteration) + ".png");
					cv::imwrite(out_path_undist_img.string(), undist_true_image); 

					data_manager->set_true_image(undist_true_image);
					camera_matrix = camera_matrix_new.clone();
					dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1); // Reset distortion coefficients since image is now undistorted
					_have_fisheye = false; // Fisheye corrections no longer needed
					_calc_IO = false; // Avoid recalculating IO repeatedly
				}
			}
			else {
				// Terminate program if matching tool is unavailable
				log_printer->append(TAG + "cannot continue, no matching available!");
				terminate_program("cannot continue, no matching available!");
			}
		}

		// Convert estimated pose of object relative to camera to camera pose relative to object system
		cv::Mat tVecCam, rMatCam;
		
		// If rotation is provided as a 3-vector (rVec), convert to 3x3 rotation matrix (rMat)
		if (rMatObj.rows == 1 || rMatObj.cols == 1) {
			//std::cout << "run Rodriquez" << std::endl;
			cv::Mat rotM;
			cv::Rodrigues(rMatObj, rotM);
			rMatObj = rotM.clone();
		}

		// Set rotation and translation in data manager for further processing
		data_manager->set_rotM(rMatObj);
		data_manager->set_X0_to_BBox(tVecObj.at<double>(0), tVecObj.at<double>(1), tVecObj.at<double>(2));

		// Calculate view frustum based on bounding box and updated camera pose
		data_manager->get_frustum()->calculate_view_frustum();

		
		// ------------------------------
		// Output Extrinsics / intrinsics
		// ------------------------------

		if (iteration == _final_iteration_number) {

			std::string out_extr = "";
			std::string out_intr_px = "";
			std::string out_intr_mm = "";
			std::string out_dist = "";
			
			cv::Mat rVecObj;
			cv::Rodrigues(rMatObj, rVecObj);
			
			double* ptr_rVecObj = (double*)(rVecObj.data);
			double* ptr_tVecObj = (double*)(tVecObj.data);
			
			// get standard deviations for extrinsics
			// https://forum.opencv.org/t/unit-of-rvecs-stddeviationsextrinsics/4266 --> rvec in radiants
			if (!stdDevObj_Ext.empty()) {
				cv::Mat stdDevObj_rvec = stdDevObj_Ext(cv::Rect(0, 0, 1, 3)); // r1 r2 r3 t1 t2 t3
				cv::Mat stdDevObj_tvec = stdDevObj_Ext(cv::Rect(0, 3, 1, 3)); // r1 r2 r3 t1 t2 t3
				double* ptr_stdDevObj_rvec = (double*)(stdDevObj_rvec.data);
				double* ptr_stdDevObj_tvec = (double*)(stdDevObj_tvec.data);	
				out_extr.append("rotV_r0;rotV_r1;rotV_r2;transV_X0;transV_Y0;transV_Z0;std_rotV_r0;std_rotV_r1;std_rotV_r2;std_transV_X0;std_transV_Y0;std_transV_Z0\n");
				out_extr.append(
					std::to_string(ptr_rVecObj[0]) + ";" +
					std::to_string(ptr_rVecObj[1]) + ";" +
					std::to_string(ptr_rVecObj[2]) + ";" +
					std::to_string(ptr_tVecObj[0] + data_manager->get_shift_x()) + ";" +
					std::to_string(ptr_tVecObj[1] + data_manager->get_shift_y()) + ";" +
					std::to_string(ptr_tVecObj[2] + data_manager->get_shift_z()) +  ";" +
					std::to_string(ptr_stdDevObj_rvec[0]) + ";" +
					std::to_string(ptr_stdDevObj_rvec[1]) + ";" +
					std::to_string(ptr_stdDevObj_rvec[2]) + ";" +
					std::to_string(ptr_stdDevObj_tvec[0]) + ";" +
					std::to_string(ptr_stdDevObj_tvec[1]) + ";" +
					std::to_string(ptr_stdDevObj_tvec[2]));
			}
			else {
				out_extr.append("rotV_r0;rotV_r1;rotV_r2;transV_X0;transV_Y0;transV_Z0;\n");
				out_extr.append(
					std::to_string(ptr_rVecObj[0]) + ";" +
					std::to_string(ptr_rVecObj[1]) + ";" +
					std::to_string(ptr_rVecObj[2]) + ";" +
					std::to_string(ptr_tVecObj[0] + data_manager->get_shift_x()) + ";" +
					std::to_string(ptr_tVecObj[1] + data_manager->get_shift_y()) + ";" +
					std::to_string(ptr_tVecObj[2] + data_manager->get_shift_z()) );
			}

			log_printer->append("\n" + TAG + "------------- extrinsics [m] -------------");
			log_printer->append("\n" + TAG + out_extr);


			// get standard deviations for intrinsics
			double* ptr_camera_matrix_data = (double*)(camera_matrix.data);
			double* ptr_dist_coeffs_data = (double*)(dist_coeffs.data);
			double* ptr_stdDevCam_In = (double*)(stdDevCam_In.data);
			double pixSize = data_manager->get_pixel_size();

			if (ptr_stdDevCam_In != nullptr) {
				out_intr_px.append("fx;fy;cx;cy;std_fx;std_fy;std_cx;std_cy\n");
				out_intr_px.append(
					std::to_string(ptr_camera_matrix_data[0]) + ";" +
					std::to_string(ptr_camera_matrix_data[4]) + ";" +
					std::to_string(ptr_camera_matrix_data[2]) + ";" +
					std::to_string(ptr_camera_matrix_data[5]) + ";" +
					std::to_string(ptr_stdDevCam_In[0]) + ";" +
					std::to_string(ptr_stdDevCam_In[1]) + ";" +
					std::to_string(ptr_stdDevCam_In[2]) + ";" +
					std::to_string(ptr_stdDevCam_In[3]));

				out_intr_mm.append("ck_x;ck_y;xh;yh;std_ckx;std_cky;std_xh;std_yh\n");
				out_intr_mm.append(
					std::to_string(ptr_camera_matrix_data[0] * pixSize) + ";" +
					std::to_string(ptr_camera_matrix_data[4] * pixSize) + ";" +
					std::to_string(ptr_camera_matrix_data[2] * pixSize) + ";" +
					std::to_string(ptr_camera_matrix_data[5] * pixSize) + ";" +
					std::to_string(ptr_stdDevCam_In[0] * pixSize) + ";" +
					std::to_string(ptr_stdDevCam_In[1] * pixSize) + ";" +
					std::to_string(ptr_stdDevCam_In[2] * pixSize) + ";" +
					std::to_string(ptr_stdDevCam_In[3] * pixSize));

				if (ptr_dist_coeffs_data != nullptr) {
					out_dist.append("k1;k2;p1;p2;k3;std_k1;std_k2;std_p1;std_p2;std_k3\n");
					out_dist.append(
						std::to_string(ptr_dist_coeffs_data[0]) + ";" +
						std::to_string(ptr_dist_coeffs_data[1]) + ";" +
						std::to_string(ptr_dist_coeffs_data[2]) + ";" +
						std::to_string(ptr_dist_coeffs_data[3]) + ";" +
						std::to_string(ptr_dist_coeffs_data[4]) + ";" +
						std::to_string(ptr_stdDevCam_In[4]) + ";" +
						std::to_string(ptr_stdDevCam_In[5]) + ";" +
						std::to_string(ptr_stdDevCam_In[6]) + ";" +
						std::to_string(ptr_stdDevCam_In[7]) + ";" +
						std::to_string(ptr_stdDevCam_In[8]));
				}
				else {
					out_dist.append("image undistorted");
				}
			}
			else {
				out_dist.append("no image calibration");
			}
			
			log_printer->append("\n" + TAG + "------------- intrinsics [px / mm] -------------");
			log_printer->append("\n" + TAG + out_intr_px);
			log_printer->append("\n" + TAG + out_intr_mm);
			log_printer->append("\n" + TAG + out_dist);
			log_printer->append("\n" + TAG + "Reprojection Error: " + std::to_string(repro_error));


			// print 
			std::stringstream myFn(data_manager->get_filename_true_image());
			std::string segment;
			std::vector<std::string> seglist;
			while (std::getline(myFn, segment, '.')) {
				seglist.push_back(segment);
			}

			const size_t outSize = 5;
			std::string subfix[outSize] = { "EOP", "IOP_mm", "IOP_px", "dist", "repro"};
			std::string contentToPrint[outSize] = { out_extr , out_intr_mm , out_intr_px , out_dist , std::to_string(repro_error) };
			std::string outFnBasic = path.string() + "\\" + seglist.at(0) + "_";

			for (int i = 0; i < outSize; i++) {
				std::ofstream myfile(outFnBasic + subfix[i] + ".txt");
				if (myfile.is_open()) {
					myfile << contentToPrint[i];
					myfile.close();
				}
				else std::cout << "Unable to open " + outFnBasic + subfix[i] + ".txt";
			}
		}

		// while iterating, delete and re-create working directories / pointers
		if (iteration != _final_iteration_number) {
			delete pim;
			delete matching;

			// clear working directory
			fs::remove_all(data_manager->get_path_working_directory() / "myData");
			fs::remove_all(data_manager->get_path_working_directory() / "Matching");
		}
	}

	// reference image points to estimate corresponding 3D coordinates
	matching->image_points_3D_referencing(*data_manager->get_image_points_2D_ptr(), *data_manager->get_pts_synth_3D_double(), data_manager->get_true_image(), camera_matrix, dist_coeffs, rMatObj, tVecObj, data_manager->get_shift_x(), data_manager->get_shift_y(), data_manager->get_shift_z(), _output_pointcloud, data_manager->get_file_name_image_points());

	// print log file
	log_printer->print_content_disk(path.string() + "\\logfile.txt");

	// destruction
	delete log_printer;
	delete pim;
	delete data_manager;
	delete matching;

	return (EXIT_SUCCESS);
}