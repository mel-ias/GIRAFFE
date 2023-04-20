#pragma once

#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include "stdafx.h"
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include"opencv2\opencv.hpp"
#include "json.hpp"
#include <windows.h>
#include "LogfilePrinter.h"
#include "Utils.h"

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

	cv::Point3d point;
	cv::Vec3b color;

	Recolored_Point_Cloud(cv::Point3d in_point, cv::Vec3b in_color) {
		point = in_point;
		color = in_color;
	}

	cv::Point3d getPoint3d() {
		return point;
	}

	cv::Vec3b getColor() {
		return color;
	}

};

class DataManager {

public:

	// C'tor
	// set default values for member variables
	DataManager(LogFile* logFile) {
		
		// ------------
		// SET DEFAULTS
		// ------------


		// --- from main ---
		logFilePrinter = logFile;
		path_file_point_cloud = "noPointCloud.bin"; // causes a "not a pw" error
		path_working_directory = "noDir";
		name_working_directory = "unknown";
		path_directory_feature_matching_tool = "noDir";
		path_file_output_D2Net_matches = "noDir";
		path_file_output_superglue_matches = "noDir";
		path_file_batch_call_jarEllipsoid = "noDir";
		path_file_batch_call_pyD2Net = "noDir";
		path_file_batch_call_superglue = "noDir";
		path_file_point_cloud = "noFile";

		// --- from json ---
		uuid = "no_uuid";
		file_name_true_image = "noFileName";
		file_name_water_line = "noFileName";		

		// parameters for IOP / about camera
		principal_distance = 1.0f;
		pix_size = 0.00089f;
		view_angle_half_H = 30.0f;
		view_angle_half_V = 30.0f;
		
		// parameters for EOP
		x0 = 0.0f, y0 = 0.0f, z0 = 0.0f; // translation params: projC
		shift_x = 0.0f, shift_y = 0.0f, shift_z = 0.0; // shift utm values for better handling
		//z_smartphone_height = 1.50; // add height of hand-held smartphone // TODO read this from smartphone application and transfer parameter via json
		
		azimuth = roll = pitch = 0.0f; //rotation params: Euler angles
	
		
		// thresholds / uncertainity values for point cloud projection
		thresh_projtPt_distanceToProjC = 2.0f;// 1.0f;
		thresh_projPt_maxDepthPtCloud = 200.0f;
		dh = 20.0f;
		r = 20.0f;
		
		// --- others ---
		// cout.precision(3); // set precision for outputlog if required
		d2Net_scalingFactor_trueImage = 1.0; // scaling factors for image scaling, only necessary for d2net image matching, use "no scaling", i.e. factor = 1.0 by default
		d2Net_scalingFactor_synthImage = 1.0;
		superglue_scalingFactor_trueImage = 1.0;
		superglue_scalingFactor_synthImage = 1.0;
		k_for_knn = 3;
		superglue_matching_thresh = 0.1f;
		filter_matches_ransac_fisheye = 200.0f; // run solvepnoransac to find good image-to-object matches before running spatial resection -> fisheye thresh (should be high cause OCVs solvepnp does not support fisheye cams)
		filter_matches_ransac_pinhole = 8.0f; // run solvepnoransac to find good image-to-object matches before running spatial resection -> pinhole thresh (should be small)

		// -------------------
		// INSTANTIATE OBJECTS
		// -------------------
		boundingBox = new BoundingBox(logFile); // Instantiate BoundingBox to select point cloud part to be projected, provide pointer to logfile
		pts_synth_2D_double = new std::vector<Vek2d>;
		pts_synth_3D_double = new std::vector<Vek3d>;
		pts_color_RGB_int = new std::vector<Vek3i>;
		pointCloud_recolored = new std::vector<Recolored_Point_Cloud>;
		pts_waterLine_2D_double = new std::vector<cv::Point2d>;

		coord_img = nullptr;

		Rxyz = new float[9]; // use only one instance for rotation matrix. for filling, just reference it
		Rxyz[0] = 1.0f; Rxyz[1] = 0.0f; Rxyz[2] = 0.0f;
		Rxyz[3] = 0.0f; Rxyz[4] = 1.0f; Rxyz[5] = 0.0f;
		Rxyz[6] = 0.0f; Rxyz[7] = 0.0f; Rxyz[8] = 1.0f;

		Rz = new float[9]; // rotation matrix arround z-axis
		Rz[0] = 0.0f; Rz[1] = 0.0f; Rz[2] = 0.0f;
		Rz[3] = 0.0f; Rz[4] = 0.0f; Rz[5] = 0.0f;
		Rz[6] = 0.0f; Rz[7] = 0.0f; Rz[8] = 1.0f;

		// bools
		haveVegetationMask = false; // check if TGI vegetation mask have to be applied
		have_precalibrated_IOP_camMatrix, have_precalibrated_IOP_distCoeffs, have_precalibrated_IOP_calibRMSE, have_calibration_values_android = false; // check if smartphone camera is precalibrated
		have_android_rotation_matrix = false; // check if native rotM is provided from client side
		well_distributed_object_points_3D_space = false; // check if distribution of object points is sufficient to refine cameras intrinsics
		well_distributed_object_points_image_space = false; // well distributed means: in each quadric of image are matched image points
		
	}

	// D'tor
	~DataManager() {
		if (pointCloud_recolored != nullptr) { delete pointCloud_recolored; }
		if (pts_waterLine_2D_double != nullptr) { delete pts_waterLine_2D_double; }
		if (coord_img != nullptr) { delete coord_img; }
		if (pts_synth_2D_double != nullptr) { delete pts_synth_2D_double; }
		if (pts_synth_3D_double != nullptr) { delete pts_synth_3D_double; }
		if (pts_color_RGB_int != nullptr) { delete pts_color_RGB_int; }
		if (Rxyz != nullptr) { delete Rxyz; }
		if (Rz != nullptr) { delete Rz; }
		if (boundingBox != nullptr) { delete boundingBox; }	
	}


	// -----------------------------------
	// INITIALISATION DATA: READ JSON FILE
	// -----------------------------------
	void read_json_file(std::string path_file_json) {

		// READ JSON FILE
		std::ifstream i(path_file_json);
		json j;
		i >> j;
		log_readJson << "Read JSON file..." << endl;


		// get client's uuid
		if (j["uuid"] != nullptr) {
			uuid = (j.at("uuid").get<std::string>());
			log_readJson << "Set 'uuid': " << uuid << std::endl;
		}
		else
			log_readJson << "No value for 'uuid' in json." << std::endl;


		// get file name of true (master) image
		if (j["file_name"] != nullptr) {
			 file_name_true_image = j.at("file_name").get<std::string>();
			 log_readJson << "Set 'file_name' of true image: " << file_name_true_image << std::endl;
		}
		else
			log_readJson << "No value for 'file_name' in json." << std::endl;

	
		// get file name of water line
		if (j["waterline_file_name"] != nullptr) {
			file_name_water_line = j.at("waterline_file_name").get<std::string>();
			log_readJson << "Set 'file_name' of true image: " << file_name_water_line << endl;
		}
		else
			log_readJson << "No value for 'waterline_file_name' in json." << std::endl;
	

		// get IOP (if camera is calibrated and IOP are provided in json file)
		// get camera matrix
		if (j["camera_matrix"] != nullptr) {
			std::string camera_matrix_string = j.at("camera_matrix").get<std::string>();

			double camera_matrix_temp[9];
			int i = 0;
			while (i < camera_matrix_string.size()) {
				if (camera_matrix_string[i] == '[' || camera_matrix_string[i] == ']') {
					camera_matrix_string.erase(i, 1);
				}
				else {
					i++;
				}
			}

			std::stringstream camera_strs(camera_matrix_string);
			i = -1;
			while (camera_strs.good()) {
				i++;
				std::string substr;
				getline(camera_strs, substr, ',');
				camera_matrix_temp[i] = std::stod(substr);				
			}

			precalib_camera_matrix = cv::Mat(3, 3, CV_64FC1); // convert to opencv array
			int element_counter = 0; // loop over indexes and assign values
			for (int rows = 0; rows < 3; rows++) {
				for (int cols = 0; cols < 3; cols++) {
					precalib_camera_matrix.at<double>(rows, cols) = camera_matrix_temp[element_counter];
					element_counter++;
				}
			}

			have_precalibrated_IOP_camMatrix = true;
			log_readJson << "Set 'camera_matrix' from pre-calibration: " << precalib_camera_matrix << endl;
		}
		else
			log_readJson << "No value for 'camera_matrix' in json. No pre-calibrated IOP available." << std::endl;


		// get distortion coefficients
		if (j["distortion_coefficents"] != nullptr) {
			std::string distortion_coefficents_string = j.at("distortion_coefficents").get<std::string>();

			double distortion_coefficents_temp[5];
			int i = 0;
			while (i < distortion_coefficents_string.size()) {
				if (distortion_coefficents_string[i] == '[' || distortion_coefficents_string[i] == ']') {
					distortion_coefficents_string.erase(i, 1);
				}
				else {
					i++;
				}
			}

			std::stringstream dist_strs(distortion_coefficents_string);
			i = -1;
			while (dist_strs.good()) {
				i++;
				std::string substr;
				getline(dist_strs, substr, ',');		
				distortion_coefficents_temp[i] = std::stod(substr);
				
			}
	
			distortion_coefficents_android = cv::Mat(1, 5, CV_64FC1);// convert to opencv array
			int element_counter = 0; //loop over indexes and assign values
			for (int cols = 0; cols < 5; cols++) {
				distortion_coefficents_android.at<double>(0, cols) = distortion_coefficents_temp[element_counter];
				
				element_counter++;
			}

			have_precalibrated_IOP_distCoeffs = true;
			log_readJson << "Set 'distortion_coefficents' from smartphone-based pre-calibration: " << distortion_coefficents_android << endl;
		}
		else
			log_readJson << "No value for 'distortion_coefficents' in json. No pre-calibrated IOP available." << std::endl;


		if (j["calibration_rmse"] != nullptr) {
			rmse_calibration_android = std::stod(j.at("calibration_rmse").get<std::string>());
			have_precalibrated_IOP_calibRMSE = true;
			log_readJson << "Set 'calibration_rmse' from smartphone-based pre-calibration: " << rmse_calibration_android << endl;
		}
		else
			log_readJson << "No value for 'calibration_rmse' in json." << std::endl;


		// read approximate IOP
		// set focal length in mm
		if (j["focal_length_mm"] != nullptr) {
			principal_distance = std::stof(j.at("focal_length_mm").get<std::string>());
			log_readJson << "Set 'focal_length_mm' ck: " << principal_distance << " [mm]" << endl;
		}
		else
			log_readJson << "No value for 'focal_length_mm' in json." << std::endl;


		// set view_angle_x & view_angle_y
		if (j["view_angle_x"] != nullptr && j["view_angle_y"] != nullptr) {
			view_angle_half_H = std::stof(j.at("view_angle_x").get<std::string>());
			view_angle_half_V = std::stof(j.at("view_angle_y").get<std::string>());
			log_readJson << "Set (full) opening angles 'view_angle_x'/'view_angle_y' (H/V): " << view_angle_half_H << "/" << view_angle_half_V << " [°]" << endl;
		}
		else
			log_readJson << "No information about view angle horizontal and/or vertical of camera available" << std::endl;


		// get EOP (if exterior orientation is determined and EOP (both, rvec angles and tvec coordinates) are provided in json file)
		// note: not approximations! final precise values!
		if (j["rvec"] != nullptr && j["tvec"] != nullptr) {
			std::string rvec_string = j.at("rvec").get<std::string>();
			std::string tvec_string = j.at("tvec").get<std::string>();

			double rvec_prev_temp[3], tvec_prev_temp[3];
			int i = 0;
			while (i < rvec_string.size()) {
				if (rvec_string[i] == '[' || rvec_string[i] == ']') {
					rvec_string.erase(i, 1);
				}
				else {
					i++;
				}
			}

			i = 0;
			while (i < tvec_string.size()) {
				if (tvec_string[i] == '[' || tvec_string[i] == ']') {
					tvec_string.erase(i, 1);
				}
				else {
					i++;
				}
			}

			std::stringstream rvec_strs(rvec_string), tvec_strs(tvec_string);
			i = -1;
			while (rvec_strs.good()) {
				i++;
				std::string substr;
				getline(rvec_strs, substr, ',');
				rvec_prev_temp[i] = std::stod(substr);
			}

			i = -1;
			while (tvec_strs.good()) {
				i++;
				std::string substr;
				getline(tvec_strs, substr, ',');
				tvec_prev_temp[i] = std::stod(substr);
			}

			rvecs_prev = cv::Mat::zeros(3, 1, CV_64FC1); // convert to opencv array
			tvecs_prev = cv::Mat::zeros(3, 1, CV_64FC1); // convert to opencv array

			int element_counter = 0;
			for (int rows = 0; rows < 3; rows++) { // loop over indexes and assign values
					rvecs_prev.at<double>(rows, 0) = rvec_prev_temp[element_counter];
					tvecs_prev.at<double>(rows, 0) = tvec_prev_temp[element_counter];
					element_counter++;
			}

			log_readJson << "Set 'rvec' of EOP: " << rvecs_prev << endl;
			log_readJson << "Set 'tvec' of EOP: " << tvecs_prev << endl;
		} 
		else
			log_readJson << "No value for 'rvec' & 'tvec' in json." << std::endl;


		// read approximate EOP
		// set projection center + height of hand-held camera position
		if (j["location_UTM_easting"] != nullptr && j["location_UTM_northing"] != nullptr && j["height"] != nullptr) {
			x0 = std::stod(j.at("location_UTM_easting").get<std::string>());
			y0 = std::stod(j.at("location_UTM_northing").get<std::string>());
			z0 = std::stod(j.at("height").get<std::string>()); 

			log_readJson << "Set approx. translation val's: 'location_UTM_easting', 'location_UTM_northing', 'height': " << x0 << " [m], " << y0 << " [m], " << z0 << " [m]" << endl;			
		} 
		else
			log_readJson << "No value for 'location_UTM_easting', 'location_UTM_northing', 'height' in json." << std::endl;


		// check if Android rotation matrix is available for client rotation 
		if (j["rotationMatrix"] != nullptr) {
			std::string rotM = j.at("rotationMatrix").get<std::string>();

			int i = 0;
			while (i < rotM.size()) {
				if (rotM[i] == '[' || rotM[i] == ']') {
					rotM.erase(i, 1);
				}
				else {
					i++;
				}
			}

			std::stringstream ss(rotM);
			i = -1;
			while (ss.good()) {
				i++;
				std::string substr;
				getline(ss, substr, ',');
				Rxyz[i] = std::stof(substr);
			}
			have_android_rotation_matrix = true;
			log_readJson << "Set Android 'rotationMatrix'. Size: " << i + 1 << endl;
		}
		else {
			log_readJsonErr << "No value for 'rotation Matrix' in json. Have to calculate bounding box from Euler angles." << std::endl;
		}


		// set orientation by angles
		if (j["azimuth"] != nullptr && j["pitch"] != nullptr && j["roll"] != nullptr) {
			azimuth = std::stof(j.at("azimuth").get<std::string>());
			pitch = std::stof(j.at("pitch").get<std::string>());
			roll = std::stof(j.at("roll").get<std::string>());			
			
			log_readJson << "Set approx. rotation val's (Euler angles): 'azimuth': " << azimuth << " [°], 'pitch': " << pitch << " [°], 'roll': " << roll << " [°]" << endl;
		} 
		else
			log_readJson << "No value for 'azimuth', 'pitch', 'roll' in json." << std::endl;

	
		// set pix size /--> use size of smart phone sensor
		if (j["pixel_size_mm_mean"] != nullptr) {
			pix_size = std::stof(j.at("pixel_size_mm_mean").get<std::string>()); 
			log_readJson << "Set 'pixel_size_mm_mean': " << pix_size << " [mm]" << endl;
		} 
		else {
			log_readJson << "No value for 'pixel_size_mm_mean depth' in json, Set default 'pixel_size_mm_mean " << pix_size << " [mm]"  << std::endl;
		}

		
		// set "area of uncertainity". Use dh = r. 
		// loc_accuracy options: defined by resolution of google's pose uncertainity from API (height) or from android.location accuracy parameter
		// see https://developer.android.com/reference/android/location/Location.html#getAccuracy%28%29 documentation [26.11.2020]
		if (j["loc_accuracy"] != nullptr) {
			dh = std::stof(j.at("loc_accuracy").get<std::string>());
			r = dh;	
			boundingBox->set_r(r);
			boundingBox->set_dh(dh);
			log_readJson << "Set 'loc_accuracy'. Noise position location (r): " << r << " [m]" << endl;
			log_readJson << "Set 'loc_accuracy'. Noise position height (dh): " << dh << " [m]" << endl;		
		}
		else
			log_readJson << "No value for 'loc_accuracy' in json." << std::endl;


		// set depth of point cloud. for real not used because of hard definition d = 200 m.
		if (j["distance"] != nullptr) {
			thresh_projPt_maxDepthPtCloud = std::stof(j.at("distance").get<std::string>());
			log_readJson << "Set 'thresh_projPt_maxDepthPtCloud': " << thresh_projPt_maxDepthPtCloud << " [m]" << endl;
		}
		else {
			log_readJson << "No value for 'distance' in json, set default 'thresh_projPt_maxDepthPtCloud': " << thresh_projPt_maxDepthPtCloud << " [m]" << std::endl;
		}


		// set tolerance value to project points just in a defined area away from the user to avoid e.g. railings or others to be reprojected in the image
		if (j["tolerance_depth"] != nullptr) {
			thresh_projtPt_distanceToProjC = std::stof(j.at("tolerance_depth").get<std::string>());	
			log_readJson << "Set 'thresh_projtPt_distanceToProjC': " << thresh_projtPt_distanceToProjC << " [m]" << endl;
		}
		else {
			log_readJson << "No value for 'tolerance depth' in json, Set default 'thresh_projtPt_distanceToProjC': " << thresh_projtPt_distanceToProjC << " [m]" << std::endl;
		}


		// -----------------------------------
		// UPDATING DATA AFTER READ JSON FILE
		// -----------------------------------
		log_readJson << "Updating parameters..." << endl;


		 // read true image 
		std::string path_file_trueImage = path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + file_name_true_image;
		true_image = cv::imread(path_file_trueImage.c_str());
		log_readJson << "Get true image successuflly from path: " << path_file_trueImage << ". True image size: " << true_image.size() << endl;


		// read water line if given
		if (file_name_water_line != "noFileName") {
			std::string path_water_line = path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + file_name_water_line;
			std::ifstream inputStream(path_water_line);
			double x, y, z; // z=0
			char sep;
			while (inputStream >> x >> sep >> y >> sep >> z)
				pts_waterLine_2D_double->push_back(cv::Point2d(x, y));
			log_readJson << "Get 2D water line from path. Number of image points: " << pts_waterLine_2D_double->size() << endl;
		}
		else {
			log_readJson << "No waterline given" << endl;
		}
		

		// update parameters to calculate BBox
		// read view angle was originally provided by Android API that provides the full view angle 
		// source code for synthetic image rending, programmed by R. Boerner, uses only half view angle for calculation
		view_angle_half_H /= 2;
		view_angle_half_V /= 2;
		// check if view angle is to large 'cause view angles > 60° doesn't make sense, in these cases use default (half) view angles of 30.0°
		/*if (view_angle_half_H > 30.0f)
			view_angle_half_H = 30.0f;
		if (view_angle_half_V > 30.0f)
			view_angle_half_V = 30.0f;*/
		boundingBox->setViewAngle(view_angle_half_H, view_angle_half_V); //update bounding box
		log_readJson << "Update (half) view angles for BBox calculation: H: " << view_angle_half_H << " [°], V: " << view_angle_half_V << " [°]" << endl;


		// shift the horizontal component of projection centre to x0 = 0 and y = 0 to work with smaller coordinates (more efficient than using UTM values)
		// save shift values (must be applied to point cloud as well and later to restore original coordinates of 3D water levels) 
		shift_x =  x0;
		shift_y =  y0; 
		shift_z = z0;
		x0 -= shift_x;
		y0 -= shift_y;
		z0 -= shift_z;
		//z0 += z_smartphone_height; // add of camera when smartphone is held by human (default: 1.50 m)
		
		log_readJson << "Apply shift_x/shift_y/shift_z to 3D projection centre. Save shift_x/shift_y/shift_z for point cloud translation. shift_x: " << std::fixed << shift_x << " [m], shift_y: " << shift_y << " [m], shift_z: " << shift_z << " [m]" << endl; //Use std::fixed floating-point notation for formatting
		//log_readJson << "Add height of hand-held smartphone to vertical compontent of projection centre. z_smartphone_height:" << z_smartphone_height << " [m]" << endl; 
		boundingBox->set_X0_Cam_World(x0, y0, z0); //update bounding box projC

		boundingBox->setAngles(azimuth, roll, pitch); //update bounding box rotP
		if (!have_android_rotation_matrix) 
			boundingBox->calcRotM_XYZ(Rxyz); // calculate rotation matrix from Euler angles if no rotation matrix is availabe from the client

		log_readJson << "Set projection centre (x0,y0,z0) for BBox calculation: " << x0 << ", " << y0 << ", " << z0 << " [m]" << endl;
		log_readJson << "Set Euler angles (azimuth,roll,pitch) for BBox calculation: " << azimuth << ", " << roll << ", " << pitch << " [°]" << endl;


		boundingBox->setDist(thresh_projPt_maxDepthPtCloud); //set max depth of point cloud to be projected
		log_readJson << "Set max depth for points to be projected from point cloud (thresh_projPt_maxDepthPtCloud) for BBox calculation: " << thresh_projPt_maxDepthPtCloud << " [m]" << endl;

		// -----
		// when finished parameter updates --> recalculate bounding box of point cloud to be projected
		boundingBox->calcBoundingBox();
		
		// check if android calibration is valid (all bools have to be true)
		if (have_precalibrated_IOP_camMatrix && have_precalibrated_IOP_distCoeffs && have_precalibrated_IOP_calibRMSE)
			have_calibration_values_android = true;
		log_readJson << "have valid camera calibration [camera_matrix, distortion_coefficents, rmse] of client's camera" << endl;

		// print content of received json data
		printLogfile_log_readJson();

	}



	
	bool generate_batch_matching(const cv::Mat& in_true_image, const cv::Mat& in_synth_image, const int flag) {
		// check true and synth image to match
		if (in_true_image.empty() || in_synth_image.empty()) {
			log_generateBatchFile << "cannot generate batch file due to missing images" << std::endl;
			return false;
		}
		else {
			log_generateBatchFile << "generate batch file to run image matching" << std::endl;
		}

		// define file path for matching input data and results 
		std::string path_directory_myData = (this->get_path_working_directory() + "\\myData\\");
		CreateDirectoryA(LPCSTR(path_directory_myData.c_str()), NULL); //generate myData directory

		std::string path_file_imagelist_to_match = (path_directory_myData + "image_file_list.txt").c_str(); // generate imageList.txt & bat-file for d2Net
		std::string path_file_trueImage_to_match = (path_directory_myData + "true_image.jpg").c_str();
		std::string path_file_synthImage_to_match = (path_directory_myData + "synth_image.jpg").c_str();
		path_file_output_D2Net_matches = (path_directory_myData + "kpts.txt").c_str();
		path_file_output_superglue_matches = (path_directory_myData + "kpts.txt").c_str();

		log_generateBatchFile << "path image_file_list.txt: " << path_file_imagelist_to_match << endl;

		if (flag == D2NET) {
			// check if max_edge and max_sum_edges of trueImage and synthImage fit the requirements from d2net
			// otherwise, reduce image size (half sized currently)
			// calculate scaling factor
			d2Net_scalingFactor_trueImage = static_cast<double>(max_sum_edges) / (in_true_image.size().width + in_true_image.size().height); //0.5; 
			d2Net_scalingFactor_synthImage = static_cast<double>(max_sum_edges) / (in_synth_image.size().width + in_synth_image.size().height); //0.5
			std::cout << "d2net: scaling factors, true image: " << d2Net_scalingFactor_trueImage << ", synth image: " << d2Net_scalingFactor_synthImage << endl;
			log_generateBatchFile << "d2net: scaling factors, true image: " << d2Net_scalingFactor_trueImage << ", synth image: " << d2Net_scalingFactor_synthImage << endl;

			// apply scaling factor
			cv::Mat trueImage_scaled, synthImage_scaled;
			cv::resize(in_true_image, trueImage_scaled, cv::Size(), d2Net_scalingFactor_trueImage, d2Net_scalingFactor_trueImage);
			cv::resize(in_synth_image, synthImage_scaled, cv::Size(), d2Net_scalingFactor_synthImage, d2Net_scalingFactor_synthImage);
			// write images to myData path
			cv::imwrite(path_file_trueImage_to_match, trueImage_scaled);
			cv::imwrite(path_file_synthImage_to_match, synthImage_scaled);
		}
		else if (flag == SUPERGLUE) {
			superglue_scalingFactor_trueImage = static_cast<double>(max_img_size) / (in_true_image.size().width); //0.5; 
			superglue_scalingFactor_synthImage = static_cast<double>(max_img_size) / (in_synth_image.size().width); //0.5
			std::cout << "superglue: scaling factors, true image: " << superglue_scalingFactor_trueImage << ", synth image: " << superglue_scalingFactor_synthImage << endl;
			log_generateBatchFile << "d2net: scaling factors, true image: " << superglue_scalingFactor_trueImage << ", synth image: " << superglue_scalingFactor_synthImage << endl;

			// apply scaling factor
			cv::Mat trueImage_scaled, synthImage_scaled;
			cv::resize(in_true_image, trueImage_scaled, cv::Size(), superglue_scalingFactor_trueImage, superglue_scalingFactor_trueImage);
			cv::resize(in_synth_image, synthImage_scaled, cv::Size(), superglue_scalingFactor_synthImage, superglue_scalingFactor_synthImage);
			// write images to myData path
			cv::imwrite(path_file_trueImage_to_match, in_true_image);
			cv::imwrite(path_file_synthImage_to_match, in_synth_image);
		}
		else {
			// TODO write error message 
			return false;
		}


		// check if image data have been stored 
		while (Utils::calculateFileSize(path_file_trueImage_to_match) == NULL || Utils::calculateFileSize(path_file_synthImage_to_match) == NULL || Utils::calculateFileSize(path_file_trueImage_to_match) < 1 || Utils::calculateFileSize(path_file_synthImage_to_match) < 1) {
			// do nothing as long as image data have not been stored
		}

		log_generateBatchFile << "file size real image: " << Utils::calculateFileSize(path_file_trueImage_to_match) << ", file size virtual image: " << Utils::calculateFileSize(path_file_synthImage_to_match) << endl;

		// open filestreams for 'image_list_file' and batch file 
		std::ofstream myFileImageList;
		myFileImageList.open(path_file_imagelist_to_match); // store in WD
		myFileImageList << path_file_trueImage_to_match << endl;
		myFileImageList << path_file_synthImage_to_match << endl;

		myFileImageList.close();

		while (Utils::calculateFileSize(path_file_imagelist_to_match) == NULL) {
			// do nothing as long as file is not available
		}


		if (flag == D2NET) {
			// generate batch file 
			path_file_batch_call_pyD2Net = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\d2net\\myBatchD2Net-1.bat").c_str();
			std::string path_d2net = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\d2net").c_str();

			std::ofstream myD2NetBatchFile;
			myD2NetBatchFile.open(path_file_batch_call_pyD2Net); // store in WD
			myD2NetBatchFile
				<< "cd " << path_d2net
				<< " &"
				<< " %windir%\\System32\\cmd.exe /k"
				<< " \"\"C:\\Users\\Mela\\miniconda3\\Scripts\\activate.bat\" py38d2net & python --version & python.exe " << path_d2net << "\\extract_features.py" //TODO make path to anaconda env generic!
				<< " --image_list_file " << path_file_imagelist_to_match
				<< " --max_edge " << max_edge
				<< " --max_sum_edges " << max_sum_edges
				<< " & exit() & cd " << path_directory_myData << "\"" << endl;
			myD2NetBatchFile.close(); // store in WD

			return true;
		}
		else if (flag == SUPERGLUE) {
			// TODO: Work in Progress
			// generate batch file 
			path_file_batch_call_superglue = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\superglue\\myBatchSuperglue.bat").c_str();
			std::string path_superglue = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\superglue").c_str();

			// open filestreams for 'image_list_file' and batch file 
			std::ofstream myFileImageList;
			myFileImageList.open(path_file_imagelist_to_match); // store in WD
			myFileImageList << path_file_trueImage_to_match << " " << path_file_synthImage_to_match << endl;
			myFileImageList.close();

			
			std::ofstream mySuperglueBatchFile;
			mySuperglueBatchFile.open(path_file_batch_call_superglue); 
			mySuperglueBatchFile << "cd " << path_superglue
				<< " &"
				<< " %windir%\\System32\\cmd.exe /k"
				<< " \"\"C:\\Users\\Mela\\miniconda3\\Scripts\\activate.bat\" py38superglue & python --version & python.exe " << path_superglue << "./match_pairs.py" //TODO make path to anaconda env generic!
				<< " --resize " << std::to_string(max_img_size)
				<< " --max_length 100000 --keypoint_threshold 0.005 --sinkhorn_iterations 20 --match_threshold " << std::to_string(superglue_matching_thresh) <<  " --show_keypoints --superglue outdoor --max_keypoints -1 --nms_radius 4 --viz"
				<< " --input_dir " << path_directory_myData
				<< " --input_pairs " << path_file_imagelist_to_match
				<< " --output_dir " << path_directory_myData
				<< " & exit() & cd " << path_directory_myData << "\"" << endl;



			// --input_dir assets/lemko_sample_images/ --input_pairs assets/lemko_sample.txt --output_dir dump_match_pairs_lemko --viz



		}
		else {
			// TODO: error message 
			return false;
		}

	}

	
	// -----
	// ENUMS
	// -----
	// added enums to select matching approach (D2NET/SG)
	enum Flags_Matching_Approach {
		D2NET, SUPERGLUE
	};

	enum FILTER_APPS {
		APPLY_WALLIS,
		APPLY_GAUSSIAN,
		APPLY_FASTNLMEANS_COLOR,
		APPLY_BILATERAL_COLOR
	};








	// --------
	// PRINTERS 
	// --------
	void printLogfile_log_readJson() {
		std::string line, lineErr;
		
		logFilePrinter->append(""); // return one line
		while (std::getline(log_readJson, line)) {
			logFilePrinter->append("JSON:\t\t" + line);
		}
		
		int number_of_lines = 0;
		while (std::getline(log_readJsonErr, lineErr)) {
			logFilePrinter->append("JSON_Err:\t" + lineErr);
			number_of_lines++;
		}

		if (number_of_lines == 0)
			logFilePrinter->append("JSON_Err:\tno errors");
	}

	void printLogfile_log_generateBatchFile() {
		std::string line;

		logFilePrinter->append(""); // return one line
		while (std::getline(log_generateBatchFile, line)) {
			logFilePrinter->append("GenBatFile:\t" + line);
		}	
	}


	// -----------------
	// GETTERS / SETTERS 
	// -----------------

	// PATHS
	std::string get_path_working_directory() { return path_working_directory; }	
	std::string get_name_working_directory() { return name_working_directory; }
	std::string get_path_file_pointcloud() { return path_file_point_cloud; }
	std::string getPathOutputFile_D2Net() { return path_file_output_D2Net_matches; }
	std::string getPathOutputFile_Superglue() { return path_file_output_superglue_matches; }
	std::string getPathBatchFile_D2Net() { return path_file_batch_call_pyD2Net; }
	std::string getPathBatchFile_Superglue() { return path_file_batch_call_superglue; }
	std::string getExeDirectory() { return path_directory_feature_matching_tool; }
	std::string getPathBatchFile_EllipsoidJar() { return path_file_batch_call_jarEllipsoid; }

	void set_path_working_directory(std::string path_dir) { path_working_directory = path_dir; }
	void set_working_directory_name(std::string wD_name) { name_working_directory = wD_name; }
	void set_path_file_pointcloud(std::string path) { path_file_point_cloud = path; } // get/set path of point cloud used for synthetic image rendering (pointcloud.pw file)
	void setDirectoryExecutable(std::string value) { path_directory_feature_matching_tool = value; }
	void setPath_to_ellipsoid_jar_batch(std::string _path) { path_file_batch_call_jarEllipsoid = _path; }
	
	// LOGFILE
	LogFile* getLogFilePrinter() { return logFilePrinter; } // get logfile
	

	// IMAGE MANAGEMENT 
	// getter
	cv::Mat& get_synth_image() { return synth_image; } //get synth image
	cv::Mat& get_true_image() { return true_image; } //get true image
	CoordinateImage* get_coordinate_image() { return coord_img; } // get coordinate image of projected point cloud
	cv::Mat get_mask_real_image_veg_TGI() { return mask_veggi_TGI_trueImg; } // get information about vegetation mask of true image
	cv::Mat get_mask_synth_image_veg_TGI() { return mask_veggi_TGI_synthImg; } // get information about vegetation mask of synth image
	std::string get_filename_true_image() { return file_name_true_image; }	// get file name of true (master) image (set during read out of JSON meta data file)
	cv::Size get_size_true_image() { return true_image.size(); } // get image size of true image

	//setter
	void set_synth_image(cv::Mat& synth_img) { synth_img.copyTo(synth_image); } //set synth image
	void set_true_image(cv::Mat& true_img) { true_img.copyTo(true_image); } //set true image 
	void set_coordinate_image(int column, int row) { coord_img = new CoordinateImage(column, row); }	// set coordinate image of projected point cloud
	void setMaskTGIVegetationRealImage(cv::Mat mask) { mask.copyTo(mask_veggi_TGI_trueImg); } // set vegetation mask true image
	void setMaskTGIVegetationSynthImage(cv::Mat mask) { mask.copyTo(mask_veggi_TGI_synthImg); } // set vegetation mask synth image

	bool have_mask_veggi_TGI_trueImg() {
		if (mask_veggi_TGI_trueImg.empty())
			return false;
		else
			return true;
	}

	bool have_mask_veggi_TGI_synthImg() {
		if (mask_veggi_TGI_synthImg.empty())
			return false;
		else
			return true;
	}


	// POINT CLOUD MANAGEMENT
	// getter point cloud / projected points
	std::vector<Vek2d>* get_pts_synth_2D_double() { return pts_synth_2D_double; }
	std::vector<Vek3d>* get_pts_synth_3D_double() { return pts_synth_3D_double; }
	std::vector<Vek3i>* get_pts_color_RGB_int() { return pts_color_RGB_int; }
	std::vector<Recolored_Point_Cloud>* get_point_cloud_recolored() { return pointCloud_recolored; }

	// getter/setter utm shift
	double get_shift_x() { return shift_x; }
	double get_shift_y() { return shift_y; }
	double get_shift_z() { return shift_z; }
	void set_shift_x(double x) { shift_x = x; }
	void set_shift_y(double y) { shift_y = y; }
	void set_shift_z(double z) { shift_z = z; }

	// getter BoundingBox 
	BoundingBox* getBoundingBox() { return boundingBox; }


	// PARAMETERS	
	std::string get_uuid() { return uuid; } 	// get uuid

	std::vector<cv::Point2d>* get_water_line_image_points_2D_ptr() { return pts_waterLine_2D_double; } // get pointer to 2D water line

	// get camera attributes
	float& get_pixel_size() { return pix_size; }
	float& getOpeningAngleHorizontal() { return view_angle_half_H; }
	float& getOpeningAngleVertical() { return view_angle_half_V; }
	float& getFocalLength() { return principal_distance; }
	
	// in case of available camera information from android
	bool get_have_camera_calibration_android_cm() { return have_precalibrated_IOP_camMatrix; } // cam matrix
	bool get_have_camera_calibration_android_dc() { return have_precalibrated_IOP_distCoeffs; } // distortion coefficients
	
	cv::Mat get_camera_calibration_android_cm() { 
		std::stringstream cam_mat_string; cam_mat_string << precalib_camera_matrix;
		logFilePrinter->append(TAG + "Deliver camera matrix: " + cam_mat_string.str());
		return precalib_camera_matrix; }
	
	cv::Mat get_camera_calibration_android_dc() { 
		std::stringstream dist_mat_string; dist_mat_string << distortion_coefficents_android;
		logFilePrinter->append(TAG + "Deliver distortion coefficents: " + dist_mat_string.str());
		return distortion_coefficents_android; }
	
	double get_camera_calibration_android_rmse() { return rmse_calibration_android; }
		
	// get information about projection settings
	float& getDistance() { return thresh_projPt_maxDepthPtCloud; }
	float& getDistanceNoise() { return dh; }
	float& getWidenessNoise() { return r; }
	float& getThresholdForPointProjection() { return thresh_projtPt_distanceToProjC; }
	
	// translation / rotation parameters
	cv::Mat get_tvecs_prior() {
		std::stringstream tvec_string; tvec_string << tvecs_prev;
		logFilePrinter->append(TAG + "Deliver tvecs prior: " + tvec_string.str());
		return tvecs_prev;
	}
	cv::Mat get_rvecs_prior() {
		std::stringstream rvec_string; rvec_string << rvecs_prev;
		logFilePrinter->append(TAG + "Deliver rvecs prior: " + rvec_string.str());
		return rvecs_prev;
	}

	cv::Point3d& getProjectionCenter() { return cv::Point3d(x0, y0, z0); }

	void set_ProjectionCenter(double _x0, double _y0, double _z0) {
		x0 = _x0;
		y0 = _y0;
		z0 = _z0;

		boundingBox->set_X0_Cam_World(x0, y0, z0);
	}

	// get orientation angles
	float& getAzimuth() { return azimuth; }
	float& getRoll() { return roll; }
	float& getPitch() { return pitch; }

	// get rotation matrix
	float* getRotationMatrix() { return Rxyz; } 

	// set rotation matrix [row-major]
	void set_RotationMatrix(cv::Mat rotM) {
		Rxyz[0] = rotM.at<double>(0, 0); Rxyz[1] = rotM.at<double>(0, 1); Rxyz[2] = rotM.at<double>(0, 2);
		Rxyz[3] = rotM.at<double>(1, 0); Rxyz[4] = rotM.at<double>(1, 1); Rxyz[5] = rotM.at<double>(1, 2);
		Rxyz[6] = rotM.at<double>(2, 0); Rxyz[7] = rotM.at<double>(2, 1); Rxyz[8] = rotM.at<double>(2, 2);
	}

	// set Rz [row-major]
	//void setRz_4_BBox(cv::Mat rotM) {
	//	
	//	Rz[0] = rotM.at<double>(0, 0);		Rz[1] = rotM.at<double>(0, 1);	Rz[2] = 0;
	//	Rz[3] = -rotM.at<double>(0, 1);		Rz[4] = rotM.at<double>(0, 0);	Rz[5] = 0; //note, Ryz Rotation clockwise arount z-axis like a compass
	//	Rz[6] = 0;							Rz[7] = 0;						Rz[8] = 1;

	//	boundingBox->set_Rz(Rz);
	//}

	// MATCHING

	// get d2Net scaling factor for true and synthetic image to match max_edge_sum and max_edge
	double get_d2Net_scalingFactor_trueImage() { return d2Net_scalingFactor_trueImage; }
	double get_d2Net_scalingFactor_synthImage() { return d2Net_scalingFactor_synthImage; }

	double get_superglue_scalingFactor_trueImage() { return superglue_scalingFactor_trueImage; }
	double get_superglue_scalingFactor_synthImage() { return superglue_scalingFactor_synthImage; }

	void set_superglue_matching_thresh(float val) { 
		if (val > 0.0 && val < 1.0) {
			superglue_matching_thresh = val;
		}
		else {
			logFilePrinter->append("Invalid value for super glue matching threshold. Must be 0.0<val<1.0. Restore default (0.1)");
			superglue_matching_thresh = 0.1f;
		}
	}
	float get_superglue_matching_thresh() {	return superglue_matching_thresh; }

	void set_filter_matches_ransac_fisheye(float val) {
		if (val > 0.0) {
			filter_matches_ransac_fisheye = val;
		}
		else {
			logFilePrinter->append("Invalid value for filter_matches_ransac_fisheye threshold. Must be val > 0. Restore default (200.0)");
			filter_matches_ransac_fisheye = 200.0f;
		}
	}
	float get_filter_matches_ransac_fisheye() { return filter_matches_ransac_fisheye; }

	void set_filter_matches_ransac_pinhole(float val) {
		if (val > 0.0) {
			filter_matches_ransac_pinhole = val;
		}
		else {
			logFilePrinter->append("Invalid value for filter_matches_ransac_pinhole threshold. Must be val > 0. Restore default (8.0)");
			filter_matches_ransac_pinhole = 8.0f;
		}
	}
	float get_filter_matches_ransac_pinhole() { return filter_matches_ransac_pinhole; }



	// get/set infos about object point distribution & IO refinement (if or if not!)
	void set_well_distributed_object_points_3D_space(bool val) { well_distributed_object_points_3D_space = val; }
	bool get_well_distributed_object_points_3D_space() { return well_distributed_object_points_3D_space; }
	void set_well_distributed_object_points_image_space(bool val) { well_distributed_object_points_image_space = val; }
	bool get_well_distributed_object_points_image_space() { return well_distributed_object_points_image_space; }

	// neighbour surrounding for image fill knn
	void setKnn(size_t knn) { k_for_knn = knn; }
	size_t getKnn() { return k_for_knn; }
	



private:

	// constants 
	// ---------
	const std::string TAG = "DataManager:\t";
	// D2Net
	const uint max_edge = 1600; // set local parameters max_edge and max_sum_edges
	const uint max_sum_edges = 2400; // 2800; // caution! definition from d2net, do better not change (otherwise it will require a lot of VRAM on GPU or the calculation will take a lot of time!)

	// SUPERGLUE
	const uint max_img_size = 960;
	

	// paths to directories or files
	// -----------------------------
	std::string path_working_directory;
	std::string name_working_directory;
	std::string path_directory_feature_matching_tool;
	std::string path_file_point_cloud;
	std::string path_file_output_D2Net_matches;
	std::string path_file_output_superglue_matches;
	std::string path_file_batch_call_jarEllipsoid;
	std::string path_file_batch_call_pyD2Net;
	std::string path_file_batch_call_superglue;
	std::string file_name_true_image, file_name_water_line;

	// logoutput (gesammelt plotten)
	LogFile* logFilePrinter;
	std::stringstream log_readJson, log_readJsonErr; 
	std::stringstream log_generateBatchFile;
	
	// image data
	// ----------
	cv::Mat true_image; 
	cv::Mat synth_image;
	cv::Mat mask_veggi_TGI_trueImg, mask_veggi_TGI_synthImg; // masking vegetation via TGI

	// parameters
	// ----------
	std::string uuid;// uuid client from json
	float pix_size, dh, r, view_angle_half_H, view_angle_half_V, principal_distance; // for IOP
	cv::Mat precalib_camera_matrix;
	cv::Mat distortion_coefficents_android;
	double rmse_calibration_android;

	double x0, y0, z0; // , z_smartphone_height; //for EOP
	double shift_x, shift_y, shift_z; // enable shift of georeferenced point clouds (utm values very large numbers) 
	cv::Mat tvecs_prev, rvecs_prev; // in case of previous done exterior orientation determination
	float azimuth, roll, pitch; 
	float* Rxyz;
	float* Rz;

	float thresh_projtPt_distanceToProjC; // for projection; check if point to be projected is to close to projection centre. use 1 m distance by default 
	float thresh_projPt_maxDepthPtCloud; // max depth of point cloud to be projected starting from projection cnetre. use 200 m distance by default

	double d2Net_scalingFactor_trueImage; // scaling factors for image scaling, only necessary for 2net image matching, use "no scaling", i.e. factor = 1.0 by default
	double d2Net_scalingFactor_synthImage;
	double superglue_scalingFactor_trueImage;
	double superglue_scalingFactor_synthImage;
	float superglue_matching_thresh;
	float filter_matches_ransac_fisheye;
	float filter_matches_ransac_pinhole;
	size_t k_for_knn;

	// objects
	// -------
	BoundingBox* boundingBox; 
	CoordinateImage* coord_img;

	// vectors with data 
	// -----------------
	std::vector<Vek2d>* pts_synth_2D_double; // for matching
	std::vector<Vek3d>* pts_synth_3D_double; // for matching
	std::vector<Vek3i>* pts_color_RGB_int; 	// for matching
	std::vector<cv::Point2d>* pts_waterLine_2D_double; // water line
	std::vector<Recolored_Point_Cloud>* pointCloud_recolored; // point cloud colored

	// calculated IOP and EOP after space resection (solve PnP adjustment)
	cv::Point3d projCenter_Corr;
	cv::Vec3f eulerAngles_Corr;
	double focal_length_Corr;
	cv::Point2d principle_point_Corr;

	// bools
	// -----
	bool haveVegetationMask;	
	bool have_precalibrated_IOP_camMatrix, have_precalibrated_IOP_distCoeffs, have_precalibrated_IOP_calibRMSE, have_calibration_values_android ;
	bool have_android_rotation_matrix ;
	bool well_distributed_object_points_3D_space; // check if distribution of object points is sufficient to refine cameras intrinsics
	bool well_distributed_object_points_image_space;  // well distributed means: in each quadric of image are matched image points


};


