// VirtualImageGenerator.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include "PerspectiveImage.h"
#include "Matching.h"
#include "json.hpp"
#include "DataManager.h"
#include <windows.h>
#include <ctime>
#include "LogfilePrinter.h"
#include <filesystem>
#include "boost/filesystem.hpp"
#include "Utils.h"


const::std::string NO_TAG = "";
const::std::string TAG = "Main:\t\t";
const::std::string ERROR_TAG = "Error:\t\t";

// pointers
DataManager* data_manager;
LogFile* log_printer;
Matching* matching;
PerspectiveImage* pim;

// ToDos
// TODO -> check distribution image <-> object point correspondences, apply IOP optimization only if point correspondences are well distributed 



// -----
// FLAGS: defaults
// -----
int matching_approach = DataManager::SUPERGLUE; //  DataManager::D2NET; 
// neighbourDistances_allowed: d2net: 0.25f; superglue: 5.5 --> hardly depends on pcl densitiy!
float neighbourDistance_allowed = 5.5f; // hardly depends on point cloud density!
int final_iteration_number = 0; // remember: counter starts at '0'
bool calc_IO;
bool output_pointcloud;
bool fisheye;
double balance_undist = 0.0;

// https://stackoverflow.com/a/24386991
template<class T>
T base_name(T const& path, T const& delims = "/\\")
{
	return path.substr(path.find_last_of(delims) + 1);
}
template<class T>
T remove_extension(T const& filename)
{
	typename T::size_type const p(filename.find_last_of('.'));
	return p > 0 && p != T::npos ? filename.substr(0, p) : filename;
}



// terminate VIG.exe in case of an error and delete all pointer variables
int terminate_program(std::string error_message) {
	if (pim != nullptr)
		delete pim;
	if (data_manager != nullptr)
		delete data_manager;
	if (matching != nullptr)
		delete matching;
	if (pim != nullptr)
		delete pim;

	log_printer->append(ERROR_TAG + error_message);

	if (log_printer != nullptr)
		delete log_printer;

	return (EXIT_FAILURE);
}



// read initialisation file
void read_init_file() {
	std::string path_init = (data_manager->getExeDirectory().substr(0, data_manager->getExeDirectory().find_last_of("\\/")) + "\\init.txt").c_str();

	//check if init file exists, otherwise quit
	if (!Utils::is_file(path_init)) {
		log_printer->append(TAG + "Could not found init.txt. Please check!");
	}

	// read init.txt
	std::ifstream i(path_init);
	json j;
	i >> j;

	if (j["matching_approach"] != nullptr) {
		std::string matching_approach_str = (j.at("matching_approach").get<std::string>());
		if (matching_approach_str.compare("D2NET") == 0) {
			matching_approach = DataManager::D2NET;
		}
		else if (matching_approach_str.compare("SUPERGLUE") == 0) {
			matching_approach = DataManager::SUPERGLUE;
		}
		else if (matching_approach_str.compare("LIGHTGLUE") == 0) {
			matching_approach = DataManager::LIGHTGLUE;
		}
		else {
			terminate_program("read init.txt, no valid matching approach found. Break.");
		}
		log_printer->append(TAG + " read init.txt, matching approach: " + matching_approach_str);
	}
	else {
		terminate_program("read init.txt, no valid matching approach found. Break.");
	}

	if (j["superglue_matching_thresh"] != nullptr) {
		float superglue_matching_thresh = std::stof(j.at("superglue_matching_thresh").get<std::string>());
		data_manager->set_superglue_matching_thresh(superglue_matching_thresh);
		log_printer->append(TAG + " read init.txt, superglue_matching_thresh: " + std::to_string(superglue_matching_thresh));
	}
	else {
		terminate_program("read init.txt, no valid superglue_matching_thresh value found. Break.");
	}


	if (j["filter_matches_ransac_pinhole"] != nullptr) {
		float filter_matches_ransac_pinhole = std::stof(j.at("filter_matches_ransac_pinhole").get<std::string>());
		data_manager->set_filter_matches_ransac_pinhole(filter_matches_ransac_pinhole);
		log_printer->append(TAG + " read init.txt, filter_matches_ransac_pinhole: " + std::to_string(filter_matches_ransac_pinhole));
	}
	else {
		terminate_program("read init.txt, no valid filter_matches_ransac_pinhole value found. Break.");
	}

	if (j["filter_matches_ransac_fisheye"] != nullptr) {
		float filter_matches_ransac_fisheye = std::stof(j.at("filter_matches_ransac_fisheye").get<std::string>());
		data_manager->set_filter_matches_ransac_fisheye(filter_matches_ransac_fisheye);
		log_printer->append(TAG + " read init.txt, filter_matches_ransac_fisheye: " + std::to_string(filter_matches_ransac_fisheye));
	}
	else {
		terminate_program("read init.txt, no valid filter_matches_ransac_fisheye value found. Break.");
	}


	if (j["neighbour_distance"] != nullptr) {
		neighbourDistance_allowed = std::stof(j.at("neighbour_distance").get<std::string>());
		log_printer->append(TAG + " read init.txt, neighbour_distance: " + std::to_string(neighbourDistance_allowed));
	}
	else {
		terminate_program("read init.txt, no valid neighbour distance value found. Break.");
	}

	if (j["final_iteration_number"] != nullptr) {
		final_iteration_number = std::stoi(j.at("final_iteration_number").get<std::string>());
		log_printer->append(TAG + " read init.txt, final_iteration_number: " + std::to_string(final_iteration_number));
	}
	else {
		terminate_program("read init.txt, no valid final_iteration number value found.Break.");
	}

	if (j["calc_IO"] != nullptr && j.at("calc_IO").get<std::string>().compare("true") == 0) {
		calc_IO = true;
		log_printer->append(TAG + " read init.txt, will calc_IO");
	}
	else {
		calc_IO = false;
		log_printer->append(TAG + " read init.txt, will NOT calc_IO");
	}

	if (j["fisheye"] != nullptr && j.at("fisheye").get<std::string>().compare("true") == 0) {
		fisheye = true;
		log_printer->append(TAG + " read init.txt, will use fisheye camera model to estimate camera parameters in final-1 epoch in case of refine_iop = true");
	}
	else {
		fisheye = false;
		log_printer->append(TAG + " read init.txt, will use central perspective camera model to estimate camera parameters in final-1 epoch in case of refine_iop = true");
	}

	if (j["balance_undist"] != nullptr) {
		balance_undist = std::stod(j.at("balance_undist").get<std::string>());
		log_printer->append(TAG + " read init.txt, balance_undist: " + std::to_string(balance_undist));
	}
	else {
		terminate_program("read init.txt, no valid balance_undist value found. Break.");
	}


	if (j["output_pointcloud"] != nullptr && j.at("output_pointcloud").get<std::string>().compare("true") == 0) {
		output_pointcloud = true;
		log_printer->append(TAG + " read init.txt, will save true-image-coloured point cloud to disk");
	}
	else {
		output_pointcloud = false;
		log_printer->append(TAG + " read init.txt, will NOT save true-image-coloured point cloud to disk");
	}

	if (j["k_fill_imgs_knn"] != nullptr) {
		size_t k_fill_imgs_knn = std::stoi(j.at("k_fill_imgs_knn").get<std::string>());
		data_manager->setKnn(k_fill_imgs_knn);
		log_printer->append(TAG + " read init.txt, k_fill_imgs_knn: " + std::to_string(k_fill_imgs_knn));
	}
	else {
		terminate_program("read init.txt, no valid knn value found. Break.");
	}

	if (j["conda_path"] != nullptr) {
		std::string path_conda_env = j.at("conda_path").get<std::string>();
		data_manager->set_path_conda_env(path_conda_env); // 5. set and read json. using dataManager for reading and storage 
		log_printer->append(TAG + "Set path to conda env");
	}

	if (j["conda_name"] != nullptr) {
		std::string name_conda_env = j.at("conda_name").get<std::string>();
		data_manager->set_name_conda_env(name_conda_env); // 5. set and read json. using dataManager for reading and storage 
		log_printer->append(TAG + "Set name of conda env");
	}

}


// arguments: 1) point_cloud.pw 2) json_file.json
int main(int argc, char** argv)
{
	// init log_file printer and data manager first
	log_printer = new LogFile();
	data_manager = new DataManager(log_printer);	// 2. init dataManager for data storage and managment // using like interface
	log_printer->append(TAG + "initialised 'logFilePrinter' and 'dataManager'");

	// First things first: check if necessary arguments are valid, else return before do anything!
	if (argc < 2) {
		terminate_program("need more arguments!\n -i filePathPointCloud\\poincloud.pw -j filePathJson\\jsonFile.json");
	}

	log_printer->append(NO_TAG +
		"-----------------------------" + "\n" +
		"START VIRTUAL-IMAGE-GENERATOR" + "\n" +
		"-----------------------------");






	// -------------------
	// READ ARGUMENTS ARGV 
	// -------------------
	// interpret input arguments from cmd line
	std::stringstream s;
	std::string path_file_pointcloudPW, path_file_jsonTxt, path_conda_env, name_conda_env;
	for (int i = 1; i < argc; ++i) {
		char check = argv[i][1];
		switch (check) {
		case 'i': path_file_pointcloudPW = argv[i + 1];
			if (argv[i + 1][0] == '"') {
				int na = path_file_pointcloudPW.find_first_of("\"");
				int nb = path_file_pointcloudPW.find_last_of("\"");
				path_file_pointcloudPW = path_file_pointcloudPW.substr(na + 1, nb - na - 1);
			}

			log_printer->append(TAG + "Read ARGV. Set path to point cloud (PW-file): " + path_file_pointcloudPW);
			data_manager->set_path_file_pointcloud(path_file_pointcloudPW); // set path point cloud (input)

			i += 1;
			break;

		case 'j': path_file_jsonTxt = argv[i + 1];
			if (argv[i + 1][0] == '"') {
				int na = path_file_jsonTxt.find_first_of("\"");
				int nb = path_file_jsonTxt.find_last_of("\"");
				path_file_jsonTxt = path_file_jsonTxt.substr(na + 1, nb - na - 1); // set path of JSON file (jsonTxt)
			}

			log_printer->append(TAG + "Read ARGV. Set path to json (TXT-file): " + path_file_jsonTxt);
			data_manager->read_json_file(path_file_jsonTxt); // 5. set and read json. using dataManager for reading and storage 
			log_printer->append(TAG + "Read json file");

			i += 1;
			break;
		}
	}


	// ------------------
	// SET UP DIRECTORIES
	// ------------------
	// create results directory (used for all results of all requests) + working dir using local time stamp as identifier
	// get path to working directory and script for feature matching

	// get time stamp
	//char localTime[20];
	//time_t now = time(0);
	//strftime(localTime, 20, "%Y-%m-%d_%H-%M-%S", localtime(&now)); // current date/time based on current system

	//09.08.22 - Update: Use Json-FileName as WorkingDirName



	std::string wD_name = remove_extension(base_name(path_file_jsonTxt));
	std::string path_dir_result = Utils::get_working_dir() + "\\" + "Result" + "\\" + wD_name;
	std::string path(path_dir_result);

	/*
	 * i starts at 2 as that's what you've hinted at in your question
	 * and ends before 10 because, well, that seems reasonable.
	 */
	 // check if working Dir already exist, if so, re-create one with a number, max allowed: 10
	uint dirCounter = 0;
	for (int i = 2; boost::filesystem::exists(path) && i < 10; ++i) {
		std::stringstream ss;
		ss << path_dir_result << "(" << i << ")";
		path = ss.str();
		dirCounter = i;
	}

	//std::string path_dir_result = Utils::get_working_dir() + "\\" + "Result" + "\\" + remove_extension(base_name(path_file_jsonTxt)) + "\\";
	if (!boost::filesystem::create_directories(path) || dirCounter == 9)
		log_printer->append(TAG + "could not create output directory");
	data_manager->set_path_working_directory(path); // set working directory in dataManager
	data_manager->set_working_directory_name(wD_name);
	data_manager->setDirectoryExecutable(argv[0]);

	log_printer->append(TAG + "working dir: " + path);
	log_printer->append(TAG + "script dir: " + std::string(argv[0]));

	// ------------------
	// READ INIT.TXT FILE 
	// ------------------
	read_init_file();


	// ----------------
	// START PROCESSING
	// ----------------

	// run image synthesis, image-to-object registration (try to refine parameters, if repro similar, stop and continue ), water level extraction in iterative way (if final_iteration_number > 0)
	cv::Mat camera_matrix, dist_coeffs, tVecObj, rMatObj, stdDevCam_In, stdDevObj_Ext;
	float repro_error;

	for (int iteration = 0; iteration <= final_iteration_number; ++iteration) {

		log_printer->append(TAG + "------------------ ITERATION " + std::to_string(iteration) + "/" + std::to_string(final_iteration_number) + "---------------------");

		//init FLAGS
		Matching::Flags_resec flags_matching = Matching::Flags_resec::CALC_EO_IO; //init

		
		// first iteration, fix IO + distC and optimise EO only
		if (calc_IO && iteration > 1) { // flag for IOP refinement set and not first iteration, refine IO
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

		// c) generate batch call to perfrom image matching (wait command to ensure that batch call has been fully generated)
		while (!data_manager->generate_batch_matching(true_img_4matching, synth_img_4matching, matching_approach)) {
			log_printer->append(TAG + "wait for generation batch file for image matching .");
		};
		data_manager->printLogfile_log_generateBatchFile(); // print log


		// init IO
		// prepare camera matrix + dist_coeffs using approximations from init file (use of pre-calib cams removed because of diverse camera models; instead use undistorted images)
		double focal_length_px = data_manager->getFocalLength() / data_manager->get_pixel_size();
		cv::Point2d center = cv::Point2d(data_manager->get_true_image().cols / 2, data_manager->get_true_image().rows / 2); // calc image center for initial principle point				
		camera_matrix = cv::Mat(3, 3, CV_64FC1);
		camera_matrix = (cv::Mat_<double>(3, 3) << focal_length_px, 0, center.x, 0, focal_length_px, center.y, 0, 0, 1);

		// dist_coeffs: fisheye model uses less coeffs than central perspective
		dist_coeffs = (fisheye) ? cv::Mat::zeros(4, 1, CV_64FC1) : cv::Mat::zeros(5, 1, CV_64FC1);


		// init EO
		// note: solvePnPRansac can be used with a guess of extrinsics (tVec, rVec), but tVec, rVec are the rot and trans of the object in camera coordiante system, not viceversa! Thus, convert before use!
		// convert camera pose in object system to object system in relation to camera pose
		tVecObj = cv::Mat();
		rMatObj = cv::Mat(3, 3, CV_64FC1);

		tVecObj.push_back(data_manager->getProjectionCenter().x);
		tVecObj.push_back(data_manager->getProjectionCenter().y);
		tVecObj.push_back(data_manager->getProjectionCenter().z);

		float* rotM_ptr = data_manager->getRotationMatrix();		// receive pointer rotation matrix, 9 elements; (row, column)
		rMatObj.at<double>(0, 0) = rotM_ptr[0]; rMatObj.at<double>(0, 1) = rotM_ptr[3]; rMatObj.at<double>(0, 2) = rotM_ptr[6];
		rMatObj.at<double>(1, 0) = rotM_ptr[1]; rMatObj.at<double>(1, 1) = rotM_ptr[4]; rMatObj.at<double>(1, 2) = rotM_ptr[7];
		rMatObj.at<double>(2, 0) = rotM_ptr[2]; rMatObj.at<double>(2, 1) = rotM_ptr[5]; rMatObj.at<double>(2, 2) = rotM_ptr[8];

		// Important! Convert view mode
		cv::transpose(rMatObj, rMatObj); // transpose R
		tVecObj = -rMatObj * tVecObj; // calc T




		// 9. Initialisation of matching 
		// start image-2-geometry intersection
		matching = new Matching();
		matching->init(data_manager);

		// 10. spatial-resection in case of need (no fix of intr and extr parameters)
		if (flags_matching != Matching::FIXED_EO_IO) {

			// init vectors for resulting matches between synth and real  image
			std::vector<cv::Point3d> matched_object_points;
			std::vector<cv::Point2d> matched_image_points_real, matched_image_points_synth;


			log_printer->append(TAG +
				"-----------------------------------------" + "\n" +
				"START FEATURE DETECTION / MATCHING : " + std::to_string(matching_approach) + "\n" +
				"-----------------------------------------");

			//VSFM removed because of legacy
			std::string path_matching_out, path_matching_batch;
			double scale_true_img, scale_synth_img;
			if (matching_approach == DataManager::D2NET) {
				path_matching_batch = data_manager->getPathBatchFile_D2Net().c_str();
				path_matching_out = data_manager->getPathOutputFile_D2Net().c_str();
				scale_true_img = data_manager->get_d2Net_scalingFactor_trueImage();
				scale_synth_img = data_manager->get_d2Net_scalingFactor_synthImage();
			}
			else if (matching_approach == DataManager::SUPERGLUE) {
				path_matching_batch = data_manager->getPathBatchFile_Superglue().c_str();
				path_matching_out = data_manager->getPathOutputFile_Superglue().c_str();
				scale_true_img = data_manager->get_superglue_scalingFactor_trueImage();
				scale_synth_img = data_manager->get_superglue_scalingFactor_synthImage();
			}
			else if (matching_approach == DataManager::LIGHTGLUE) {
				path_matching_batch = data_manager->getPathBatchFile_Lightglue().c_str();
				path_matching_out = data_manager->getPathOutputFile_Lightglue().c_str();
				scale_true_img = data_manager->get_lightglue_scalingFactor_trueImage();
				scale_synth_img = data_manager->get_lightglue_scalingFactor_synthImage();
			}
			else {
				log_printer->append(TAG + "cannot continue, no matching available!");
				terminate_program("cannot continue, no matching available!");
			}


			// run external matching tool
			if (Utils::run_batch_file(path_matching_batch)) {
				log_printer->append(TAG + "matching successful, read inliers passing fundamental test...");

				// wait until images are saved and path of visual sfm output becomes valid
				while (Utils::calculateFileSize(path_matching_out) == NULL || Utils::calculateFileSize(path_matching_out) < 1) {}

				// run post-processing of visual sfm inlier data
				// calculate interior and exterior camera parameters, necessary for transformation into object space
				matching->loadMatches(
					path_matching_out,
					data_manager->get_true_image(),
					data_manager->get_synth_image(),
					*data_manager->get_water_line_image_points_2D_ptr(),
					*data_manager->get_pts_synth_2D_double(),
					*data_manager->get_pts_synth_3D_double(),
					matched_object_points,
					matched_image_points_real,
					matched_image_points_synth,
					scale_true_img,
					scale_synth_img,
					matching_approach,
					neighbourDistance_allowed);



				

				// perform spatial resection using these points for camera and exterior orientation determination, check if values for camera are valid and available
				repro_error = matching->enhanced_spatial_resection(
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
					fisheye);


				// if IOP calculated, undistort image to bring synthetic and true image more in same perspective and to enhance super glue
				if (flags_matching == Matching::CALC_EO_IO) { // in the before last iteration do undistortion and use the undistored image in the last epoch
					cv::Mat undist_true_image;
					cv::Mat camera_matrix_new = camera_matrix.clone();
					if (fisheye) {
						cv::Mat E = cv::Mat::eye(3, 3, cv::DataType<double>::type);
						cv::Mat map1;
						cv::Mat map2;
						cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, data_manager->get_size_true_image(), E, camera_matrix_new, balance_undist);
						cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, E, camera_matrix_new, data_manager->get_size_true_image(), CV_16SC2, map1, map2);
						cv::remap(data_manager->get_true_image(), undist_true_image, map1, map2, cv::INTER_LINEAR, CV_HAL_BORDER_CONSTANT);
					}
					else {
						camera_matrix_new = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, data_manager->get_size_true_image(), balance_undist);
						cv::undistort(data_manager->get_true_image(), undist_true_image, camera_matrix, dist_coeffs);
					}
					cv::imwrite(data_manager->get_path_working_directory() + "\\" + "undist_test_" + std::to_string(iteration) + ".png", undist_true_image); // Test out
					data_manager->set_true_image(undist_true_image);
					camera_matrix = camera_matrix_new.clone();
					dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1); // camera_matrix is fine but distortion needs to be reset cause there is no distortion anymore
					fisheye = false; // after correction no longer consider fisheye
					calc_IO = false; // not correct again and again
				}
			}
			else {
				// in case of missing matching tool, terminate program
				log_printer->append(TAG + "cannot continue, no matching available!");
				terminate_program("cannot continue, no matching available!");
			}
		}



		// convert estimated pose of object in relation to camera system to camera pose in relation to object system 
		// 3-vector to 9-rotM, only necessary if rvec is a 3x1/1x3 vec and not already the 3x3 rotM
		cv::Mat tVecCam, rMatCam;
		// calc rotM if rVec is provided
		if (rMatObj.rows == 1 || rMatObj.cols == 1) {
			//std::cout << "run Rodriquez" << std::endl;
			cv::Mat rotM;
			cv::Rodrigues(rMatObj, rotM);
			rMatObj = rotM.clone();
		}

		// convert to get cameraPose
		//cv::transpose(rMatObj, rMatCam);
		//tVecCam = -rMatCam * tVecObj;
		data_manager->set_RotationMatrix(rMatObj);
		data_manager->set_ProjectionCenter(tVecObj.at<double>(0), tVecObj.at<double>(1), tVecObj.at<double>(2));
		data_manager->getBoundingBox()->calcBoundingBox();

		

		// ------------------------------
		// Output Extrinsics / intrinsics
		// ------------------------------

		if (iteration == final_iteration_number) {



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


			// print section 
			// -------------
			// get filename of image to generate a nice output file name for the orientation parameters
			std::stringstream myFn(data_manager->get_filename_true_image());
			std::string segment;
			std::vector<std::string> seglist;
			while (std::getline(myFn, segment, '.')) {
				seglist.push_back(segment);
			}

			const size_t outSize = 5;
			std::string subfix[outSize] = { "EOP", "IOP_mm", "IOP_px", "dist", "repro"};
			std::string contentToPrint[outSize] = { out_extr , out_intr_mm , out_intr_px , out_dist , std::to_string(repro_error) };
			std::string outFnBasic = path + "\\" + seglist.at(0) + "_";

			for (int i = 0; i < outSize; i++) {
				std::ofstream myfile(outFnBasic + subfix[i] + ".txt");
				if (myfile.is_open()) {
					myfile << contentToPrint[i];
					myfile.close();
				}
				else cout << "Unable to open " + outFnBasic + subfix[i] + ".txt";
			}
		}



		// while iterating, delete and re-create working directories / pointers
		if (iteration != final_iteration_number) {
			delete pim;
			delete matching;

			// clear wd
			boost::filesystem::remove_all(data_manager->get_path_working_directory() + "\\myData\\");
			boost::filesystem::remove_all(data_manager->get_path_working_directory() + "\\Matching\\");
			boost::filesystem::remove_all(data_manager->get_path_working_directory() + "\\VegetationMask\\");
		}
	}






	// 10. projection water line into object space
	log_printer->append(TAG + "Object_points_size: " + std::to_string((*data_manager->get_pts_synth_3D_double()).size()));
	log_printer->append(TAG + "Waterline_image_points_size: " + std::to_string((*data_manager->get_water_line_image_points_2D_ptr()).size()));

	matching->waterlineProjection(*data_manager->get_water_line_image_points_2D_ptr(), *data_manager->get_pts_synth_3D_double(), data_manager->get_true_image(), camera_matrix, dist_coeffs, rMatObj, tVecObj, data_manager->get_shift_x(), data_manager->get_shift_y(), data_manager->get_shift_z(), output_pointcloud);

	log_printer->print_content_disk(path + "\\logfile.txt");


	// final call destructors
	delete log_printer;
	delete pim;
	delete data_manager;
	delete matching;

	return (EXIT_SUCCESS);
}

