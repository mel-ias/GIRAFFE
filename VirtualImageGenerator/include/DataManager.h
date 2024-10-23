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
#include "vek.h"

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
		path_file_output_lightglue_matches = "noDir";
		path_file_batch_call_jarEllipsoid = "noDir";
		path_file_batch_call_lightglue = "noDir";
		path_file_point_cloud = "noFile";

		// --- from json ---
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
		Rxyz[0] = 1.0f; Rxyz[3] = 0.0f; Rxyz[6] = 0.0f;
		Rxyz[1] = 0.0f; Rxyz[4] = 1.0f; Rxyz[7] = 0.0f;
		Rxyz[2] = 0.0f; Rxyz[5] = 0.0f; Rxyz[8] = 1.0f;


		// bools
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
		log_readJson << "Read JSON file..." << std::endl;


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
			log_readJson << "Set 'file_name' of true image: " << file_name_water_line << std::endl;
		}
		else
			log_readJson << "No value for 'waterline_file_name' in json." << std::endl;

		

		// read approximate IOP
		// set focal length in mm
		if (j["focal_length_mm"] != nullptr) {
			principal_distance = std::stof(j.at("focal_length_mm").get<std::string>());
			log_readJson << "Set 'focal_length_mm' ck: " << principal_distance << " [mm]" << std::endl;
		}
		else
			log_readJson << "No value for 'focal_length_mm' in json." << std::endl;


		// set view_angle_x & view_angle_y
		if (j["view_angle_x"] != nullptr && j["view_angle_y"] != nullptr) {
			view_angle_half_H = std::stof(j.at("view_angle_x").get<std::string>());
			view_angle_half_V = std::stof(j.at("view_angle_y").get<std::string>());
			log_readJson << "Set (full) opening angles 'view_angle_x'/'view_angle_y' (H/V): " << view_angle_half_H << "/" << view_angle_half_V << " [°]" << std::endl;
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

			log_readJson << "Set 'rvec' of EOP: " << rvecs_prev << std::endl;
			log_readJson << "Set 'tvec' of EOP: " << tvecs_prev << std::endl;
		} 
		else
			log_readJson << "No value for 'rvec' & 'tvec' in json." << std::endl;


		// read approximate EOP
		// set projection center + height of hand-held camera position
		if (j["location_UTM_easting"] != nullptr && j["location_UTM_northing"] != nullptr && j["height"] != nullptr) {
			x0 = std::stod(j.at("location_UTM_easting").get<std::string>());
			y0 = std::stod(j.at("location_UTM_northing").get<std::string>());
			z0 = std::stod(j.at("height").get<std::string>()); 

			log_readJson << "Set approx. translation val's: 'location_UTM_easting', 'location_UTM_northing', 'height': " << x0 << " [m], " << y0 << " [m], " << z0 << " [m]" << std::endl;
		} 
		else
			log_readJson << "No value for 'location_UTM_easting', 'location_UTM_northing', 'height' in json." << std::endl;



		// set orientation by angles
		if (j["azimuth"] != nullptr && j["pitch"] != nullptr && j["roll"] != nullptr) {
			azimuth = std::stof(j.at("azimuth").get<std::string>());
			pitch = std::stof(j.at("pitch").get<std::string>());
			roll = std::stof(j.at("roll").get<std::string>());			
			
			log_readJson << "Set approx. rotation val's (Euler angles): 'azimuth': " << azimuth << " [°], 'pitch': " << pitch << " [°], 'roll': " << roll << " [°]" << std::endl;
		} 
		else
			log_readJson << "No value for 'azimuth', 'pitch', 'roll' in json." << std::endl;

	
		// set pix size /--> use size of smart phone sensor
		if (j["pixel_size_mm_mean"] != nullptr) {
			pix_size = std::stof(j.at("pixel_size_mm_mean").get<std::string>()); 
			log_readJson << "Set 'pixel_size_mm_mean': " << pix_size << " [mm]" << std::endl;
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
			log_readJson << "Set 'loc_accuracy'. Noise position location (r): " << r << " [m]" << std::endl;
			log_readJson << "Set 'loc_accuracy'. Noise position height (dh): " << dh << " [m]" << std::endl;
		}
		else
			log_readJson << "No value for 'loc_accuracy' in json." << std::endl;


		// set depth of point cloud. for real not used because of hard definition d = 200 m.
		if (j["distance"] != nullptr) {
			thresh_projPt_maxDepthPtCloud = std::stof(j.at("distance").get<std::string>());
			log_readJson << "Set 'thresh_projPt_maxDepthPtCloud': " << thresh_projPt_maxDepthPtCloud << " [m]" << std::endl;
		}
		else {
			log_readJson << "No value for 'distance' in json, set default 'thresh_projPt_maxDepthPtCloud': " << thresh_projPt_maxDepthPtCloud << " [m]" << std::endl;
		}


		// set tolerance value to project points just in a defined area away from the user to avoid e.g. railings or others to be reprojected in the image
		if (j["tolerance_depth"] != nullptr) {
			thresh_projtPt_distanceToProjC = std::stof(j.at("tolerance_depth").get<std::string>());	
			log_readJson << "Set 'thresh_projtPt_distanceToProjC': " << thresh_projtPt_distanceToProjC << " [m]" << std::endl;
		}
		else {
			log_readJson << "No value for 'tolerance depth' in json, Set default 'thresh_projtPt_distanceToProjC': " << thresh_projtPt_distanceToProjC << " [m]" << std::endl;
		}


		// -----------------------------------
		// UPDATING DATA AFTER READ JSON FILE
		// -----------------------------------
		log_readJson << "Updating parameters..." << std::endl;


		 // read true image 
		std::string path_file_trueImage = path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + file_name_true_image;
		true_image = cv::imread(path_file_trueImage.c_str());
		log_readJson << "Get true image successuflly from path: " << path_file_trueImage << ". True image size: " << true_image.size() << std::endl;


		// read water line if given
		if (file_name_water_line != "noFileName") {
			std::string path_water_line = path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + file_name_water_line;
			std::ifstream inputStream(path_water_line);
			double x, y, z; // z=0
			char sep;
			while (inputStream >> x >> sep >> y >> sep >> z)
				pts_waterLine_2D_double->push_back(cv::Point2d(x, y));
			log_readJson << "Get 2D water line from path. Number of image points: " << pts_waterLine_2D_double->size() << std::endl;
		}
		else {
			log_readJson << "No waterline given" << std::endl;
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
		boundingBox->set_view_angles(view_angle_half_H, view_angle_half_V); //update bounding box
		log_readJson << "Update (half) view angles for BBox calculation: H: " << view_angle_half_H << " [°], V: " << view_angle_half_V << " [°]" << std::endl;


		// shift the horizontal component of projection centre to x0 = 0 and y = 0 to work with smaller coordinates (more efficient than using UTM values)
		// save shift values (must be applied to point cloud as well and later to restore original coordinates of 3D water levels) 
		shift_x = x0;
		shift_y = y0; 
		shift_z = z0;
		x0 -= shift_x;
		y0 -= shift_y;
		z0 -= shift_z;
		//z0 += z_smartphone_height; // add of camera when smartphone is held by human (default: 1.50 m)
		
		log_readJson << "Apply shift_x/shift_y/shift_z to 3D projection centre. Save shift_x/shift_y/shift_z for point cloud translation. shift_x: " << std::fixed << shift_x << " [m], shift_y: " << shift_y << " [m], shift_z: " << shift_z << " [m]" << std::endl; //Use std::fixed floating-point notation for formatting
		//log_readJson << "Add height of hand-held smartphone to vertical compontent of projection centre. z_smartphone_height:" << z_smartphone_height << " [m]" << endl; 
		boundingBox->set_X0_Cam_World(x0, y0, z0); //update bounding box projC

		boundingBox->calculate_rotation_matrix_rzxy(azimuth, roll, pitch); //update bounding box rotP
		
		

		const float M_PI = 3.14159265358979323846;   // pi
		float Cz = static_cast<float>(cos(azimuth * M_PI / 180.0f));
		float Sz = static_cast<float>(sin(azimuth * M_PI / 180.0f));

		float Cy = static_cast<float>(cos(roll * M_PI / 180.0f));
		float Sy = static_cast<float>(sin(roll * M_PI / 180.0f));

		float Cx = static_cast<float>(cos(pitch * M_PI / 180.0f));
		float Sx = static_cast<float>(sin(pitch * M_PI / 180.0f));

		// Rxyz, performs 3 rotations in order of Rz (azi), Ry (roll) then Rx (pitch).
		// determine left axis [x, pitch]	determine up axis [y, roll]		determine forward axis [z, Azimuth]	
		Rxyz[0] = Cy * Cz;						Rxyz[3] = -Cy * Sz;					Rxyz[6] = Sy;
		Rxyz[1] = Sx * Sy * Cz + Cx * Sz;		Rxyz[4] = -Sx * Sy * Sz + Cx * Cz;	Rxyz[7] = -Sx * Cy;
		Rxyz[2] = -Cx * Sy * Cz + Sx * Sz;		Rxyz[5] = Cx * Sy * Sz + Sx * Cz;	Rxyz[8] = Cx * Cy;

		logFilePrinter->append(TAG + "DM, RotM:");
		logFilePrinter->append("\t\t" + std::to_string(Rxyz[0]) + " " + std::to_string(Rxyz[3]) + " " + std::to_string(Rxyz[6]), 4);
		logFilePrinter->append("\t\t" + std::to_string(Rxyz[1]) + " " + std::to_string(Rxyz[4]) + " " + std::to_string(Rxyz[7]), 4);
		logFilePrinter->append("\t\t" + std::to_string(Rxyz[2]) + " " + std::to_string(Rxyz[5]) + " " + std::to_string(Rxyz[8]), 4);
		logFilePrinter->append("");
		
			

		log_readJson << "Set projection centre (x0,y0,z0) for BBox calculation: " << x0 << ", " << y0 << ", " << z0 << " [m]" << std::endl;
		log_readJson << "Set Euler angles (azimuth,roll,pitch) for BBox calculation: " << azimuth << ", " << roll << ", " << pitch << " [°]" << std::endl;


		boundingBox->set_frustum_depth(thresh_projPt_maxDepthPtCloud); //set max depth of point cloud to be projected
		log_readJson << "Set max depth for points to be projected from point cloud (thresh_projPt_maxDepthPtCloud) for BBox calculation: " << thresh_projPt_maxDepthPtCloud << " [m]" << std::endl;

		// -----
		// when finished parameter updates --> recalculate bounding box of point cloud to be projected
		boundingBox->calculate_view_frustum();
		
		// print content of received json data
		printLogfile_log_readJson();

	}



	
	bool generate_batch_matching(const cv::Mat& in_true_image, const cv::Mat& in_synth_image) {
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
		path_file_output_lightglue_matches = (path_directory_myData + "kpts.txt").c_str();

		log_generateBatchFile << "path image_file_list.txt: " << path_file_imagelist_to_match << std::endl;

		// write images to myData path
		cv::imwrite(path_file_trueImage_to_match, in_true_image);
		cv::imwrite(path_file_synthImage_to_match, in_synth_image);
		


		// check if image data have been stored 
		while (Utils::calculateFileSize(path_file_trueImage_to_match) == NULL || Utils::calculateFileSize(path_file_synthImage_to_match) == NULL || Utils::calculateFileSize(path_file_trueImage_to_match) < 1 || Utils::calculateFileSize(path_file_synthImage_to_match) < 1) {
			// do nothing as long as image data have not been stored
		}

		log_generateBatchFile << "file size real image: " << Utils::calculateFileSize(path_file_trueImage_to_match) << ", file size virtual image: " << Utils::calculateFileSize(path_file_synthImage_to_match) << std::endl;
		std::cout << "image sizes (true/synth) " << in_true_image.size().width << "," << in_true_image.size().height << in_synth_image.size().width << "," << in_synth_image.size().height << std::endl;

		// open filestreams for 'image_list_file' and batch file 
		std::ofstream myFileImageList;
		myFileImageList.open(path_file_imagelist_to_match); // store in WD
		myFileImageList << path_file_trueImage_to_match << std::endl;
		myFileImageList << path_file_synthImage_to_match << std::endl;

		myFileImageList.close();

		while (Utils::calculateFileSize(path_file_imagelist_to_match) == NULL) {
			// do nothing as long as file is not available
		}


		
		
			// LIGHTGLUE
			// generate batch file 
			path_file_batch_call_lightglue = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\lightglue\\myBatchLightglue.bat").c_str();
			std::string path_lightglue = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\lightglue").c_str();


			std::ofstream myLightglueBatchFile;
			myLightglueBatchFile.open(path_file_batch_call_lightglue);

			if (path_conda_env.empty() ) {
				std::cout << "No path to conda environment given. Return." << std::endl;
				return false;
			}
			else {
				myLightglueBatchFile << "cd " << path_lightglue
					<< " &"
					<< " %windir%\\System32\\cmd.exe /k"
					<< " " << "\"\"" << path_conda_env << "\""
					<< " " << "lightglue"
					<< " " << " & python.exe"
					<< " " << path_lightglue << "./match_pairs_lightglue.py" //TODO make path to anaconda env generic!<< " --resize " << std::to_string(max_img_size)
					<< " --left_image " << path_file_trueImage_to_match
					<< " --right_image " << path_file_synthImage_to_match
					<< " --output_dir " << path_directory_myData
					<< " & exit() & cd " << path_directory_myData << "\"" << std::endl;
				std::cout << "my path to lightglue " << path_file_batch_call_lightglue << std::endl;
			}
		

	}

	


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
	std::string getPathOutputFile_Lightglue() { return path_file_output_lightglue_matches; }
	std::string getPathBatchFile_Lightglue() { return path_file_batch_call_lightglue; }
	std::string getExeDirectory() { return path_directory_feature_matching_tool; }

	void set_path_working_directory(std::string path_dir) { path_working_directory = path_dir; }
	void set_working_directory_name(std::string wD_name) { name_working_directory = wD_name; }
	void set_path_file_pointcloud(std::string path) { path_file_point_cloud = path; } // get/set path of point cloud used for synthetic image rendering (pointcloud.pw file)
	void setDirectoryExecutable(std::string value) { path_directory_feature_matching_tool = value; }

	
	// LOGFILE
	LogFile* getLogFilePrinter() { return logFilePrinter; } // get logfile

	// IMAGE MANAGEMENT 
	// getter
	cv::Mat& get_synth_image() { return synth_image; } //get synth image
	cv::Mat& get_true_image() { return true_image; } //get true image
	CoordinateImage* get_coordinate_image() { return coord_img; } // get coordinate image of projected point cloud
	std::string get_filename_true_image() { return file_name_true_image; }	// get file name of true (master) image (set during read out of JSON meta data file)
	cv::Size get_size_true_image() { return true_image.size(); } // get image size of true image

	//setter
	void set_synth_image(cv::Mat& synth_img) { synth_img.copyTo(synth_image); } //set synth image
	void set_true_image(cv::Mat& true_img) { true_img.copyTo(true_image); } //set true image 
	void set_coordinate_image(int column, int row) { coord_img = new CoordinateImage(column, row); }	// set coordinate image of projected point cloud
	


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

	std::vector<cv::Point2d>* get_water_line_image_points_2D_ptr() { return pts_waterLine_2D_double; } // get pointer to 2D water line

	// get camera attributes
	float& get_pixel_size() { return pix_size; }
	float& getOpeningAngleHorizontal() { return view_angle_half_H; }
	float& getOpeningAngleVertical() { return view_angle_half_V; }
	float& getFocalLength() { return principal_distance; }
	
	
		
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

	cv::Point3d getProjectionCenter() { return cv::Point3d(x0, y0, z0); }

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
		Rxyz[0] = rotM.at<double>(0, 0); Rxyz[3] = rotM.at<double>(0, 1); Rxyz[6] = rotM.at<double>(0, 2);
		Rxyz[1] = rotM.at<double>(1, 0); Rxyz[4] = rotM.at<double>(1, 1); Rxyz[7] = rotM.at<double>(1, 2);
		Rxyz[2] = rotM.at<double>(2, 0); Rxyz[5] = rotM.at<double>(2, 1); Rxyz[8] = rotM.at<double>(2, 2);
	}


	
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
	
	// set path/name to conda env
	void set_path_conda_env(std::string path) { path_conda_env = path; }
	void set_name_conda_env(std::string name) { name_conda_env = name; }
	std::string get_path_conda_env() { return path_conda_env; }
	std::string get_name_conda_env() { return name_conda_env; }

	// set/get file name of image points file (only file name with extension, not path!)
	void set_file_name_image_points(std::string name) { file_name_water_line = name; }
	std::string get_file_name_image_points() { return file_name_water_line; }


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
	std::string path_file_output_lightglue_matches;
	std::string path_file_output_superglue_matches;
	std::string path_file_batch_call_jarEllipsoid;
	std::string path_file_batch_call_lightglue;
	std::string file_name_true_image, file_name_water_line;
	std::string path_conda_env;
	std::string name_conda_env;

	// logoutput (gesammelt plotten)
	LogFile* logFilePrinter;
	std::stringstream log_readJson, log_readJsonErr; 
	std::stringstream log_generateBatchFile;
	
	// image data
	// ----------
	cv::Mat true_image; 
	cv::Mat synth_image;

	// parameters
	// ----------
	float pix_size, dh, r, view_angle_half_H, view_angle_half_V, principal_distance; // for IOP


	double x0, y0, z0; // , z_smartphone_height; //for EOP
	double shift_x, shift_y, shift_z; // enable shift of georeferenced point clouds (utm values very large numbers) 
	cv::Mat tvecs_prev, rvecs_prev; // in case of previous done exterior orientation determination
	float azimuth, roll, pitch; 
	float* Rxyz;


	float thresh_projtPt_distanceToProjC; // for projection; check if point to be projected is to close to projection centre. use 1 m distance by default 
	float thresh_projPt_maxDepthPtCloud; // max depth of point cloud to be projected starting from projection cnetre. use 200 m distance by default

	float filter_matches_ransac_fisheye;
	float filter_matches_ransac_pinhole;

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
	bool well_distributed_object_points_3D_space; // check if distribution of object points is sufficient to refine cameras intrinsics
	bool well_distributed_object_points_image_space;  // well distributed means: in each quadric of image are matched image points


};


