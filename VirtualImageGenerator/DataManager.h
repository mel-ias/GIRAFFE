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

	//Ctor

	DataManager(LogFile* logFile) {

		logFilePrinter = logFile;

		boundingBox = new BoundingBox(logFile);

		// init variables with default values
		path_file_point_cloud = "something.bin"; // causes a "not a pw" error
		output = "out_img";
		pixSize = 0.00089f;

		x0 = 0.0f, y0 = 0.0f, z0 = 0.0f; // X0 = origin ( kamera steht auf Scanner)
		imageSize = cv::Size(0, 0);
		azimuth = 270.0f;
		roll = 0.0f;
		pitch = 90.0f;
		
		d = 400.0f;
		
		dh = 20.0f;
		r = 20.0f;

		H = 30.0f;
		V = 30.0f;

		ck = 1.0f;

		_synth_pts_2D_double = new std::vector<Vek2d>;
		_synth_pts_3D_double = new std::vector<Vek3d>;
		_pts_farbe = new std::vector<Vek3i>;

		punktwolke_neu_eingefaerbt = new std::vector<Recolored_Point_Cloud>;
		waterlinePoints = new std::vector<cv::Point2d>;

		coord_Img = nullptr;

		// use only one instance for rotation matrix. for filling, just reference it
		Rxyz = new float[9];
		Rxyz[0] = 1.0f; Rxyz[1] = 0.0f; Rxyz[2] = 0.0f;
		Rxyz[3] = 0.0f; Rxyz[4] = 1.0f; Rxyz[5] = 0.0f;
		Rxyz[6] = 0.0f; Rxyz[7] = 0.0f; Rxyz[8] = 1.0f;

		Rz = new float[9];
		Rz[0] = 0.0f; Rz[1] = 0.0f; Rz[2] = 0.0f;
		Rz[3] = 0.0f; Rz[4] = 0.0f; Rz[5] = 0.0f;
		Rz[6] = 0.0f; Rz[7] = 0.0f; Rz[8] = 1.0f;
		//cout.precision(3); // set precision for outputlog;
	
	
	}

	// D'tor
	~DataManager() {

		if (punktwolke_neu_eingefaerbt != nullptr)
			delete punktwolke_neu_eingefaerbt;

		if (waterlinePoints != nullptr)
			delete waterlinePoints;

		if (coord_Img != nullptr)
			delete coord_Img;

		if (_synth_pts_2D_double != nullptr)
			delete _synth_pts_2D_double;

		if (_synth_pts_3D_double != nullptr)
			delete _synth_pts_3D_double;

		if (_pts_farbe != nullptr)
			delete _pts_farbe;

		if (Rxyz != nullptr)
			delete Rxyz;

		if (Rz != nullptr)
			delete Rz;

		if (boundingBox != nullptr)
			delete boundingBox;

		

	}


	// ---------------INITIALISATION DATA: READ JSON FILE ------------------ //
	void readJSONFile(std::string path_file_json) {

		// read a JSON file
		std::ifstream i(path_file_json);
		json j;
		i >> j;
		log_readJson << "Read JSON file. Define parameters: " << endl;



		// input --> nicht im JSON File beschrieben
		if (j["uuid"] != nullptr) {

			// get client's uuid
			uuid = (j.at("uuid").get<std::string>());
			log_readJson << "set clients uuid: " << uuid << std::endl;
		}
		else
			log_readJsonErr << "no uuid for client available" << std::endl;



		// input --> nicht im JSON File beschrieben
		if (j["file_name"] != nullptr) {
			 
			// define paths to image and output
			std::string pathMasterImage = path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + j.at("file_name").get<std::string>();
			std::string pathOutput = j.at("file_name").get<std::string>() + "_out";
			
			file_name_true_image = j.at("file_name").get<std::string>();

			// set input image, image size
			realImage = cv::imread(pathMasterImage.c_str());
			imageSize = realImage.size();
			log_readJson << "set input master image: " << pathMasterImage << ", size: " << imageSize << endl;

			// set output path
			output = pathOutput.c_str();
			log_readJson << "set output path: " << output << endl;
		
		}
		else
			log_readJsonErr << "no input image available!" << std::endl;

	
		// set water line
		if (j["waterline_file_name"] != nullptr) {
			 
			std::string pathWaterLine = (path_file_json.substr(0, path_file_json.find_last_of("\\/")) + "\\" + j.at("waterline_file_name").get<std::string>()).c_str();

			if (pathWaterLine == "waterline.txt") {
				std::cerr << "no waterline available" << std::endl;
				have_water_line_image_points_2D_ptr = false;
			}
			else {
				// Lade Punkte aus Txt in Vector rein
				std::ifstream inputStream(pathWaterLine);
				double x, y, z; // z=0
				char sep;
				while (inputStream >> x >> sep >> y >> sep >> z)
					waterlinePoints->push_back(cv::Point2d(x, y));

				have_water_line_image_points_2D_ptr = true;
				log_readJson << "set water line, count points: " << waterlinePoints->size() << endl;			
			}				
		}
		else
			log_readJsonErr << "no information about water line available" << std::endl;
		

		// ask for previous done camera calibration -> matrix and distortion_coefficents
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

			// convert to opencv array
			camera_matrix_android = cv::Mat(3, 3, CV_64FC1);
			int element_counter = 0;
			// Make a double loop over indexes and assign values
			for (int rows = 0; rows < 3; rows++) {
				for (int cols = 0; cols < 3; cols++) {
					camera_matrix_android.at<double>(rows, cols) = camera_matrix_temp[element_counter];
					element_counter++;
				}
			}


			have_calibration_values_android_cm = true;
			log_readJson << "Set camera matrix (Android): " << camera_matrix_android << endl;

		}
		else
			log_readJsonErr << "no information about previously done camera calibration [camera_matrix] available" << std::endl;

		
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
				getline(dist_strs, substr, ';');
				
				distortion_coefficents_temp[i] = std::stod(substr);
			
			}

				
			// convert to opencv array
			distortion_coefficents_android = cv::Mat(1, 5, CV_64FC1);
			int element_counter = 0;
			// Make a double loop over indexes and assign values
				for (int cols = 0; cols < 5; cols++) {
					distortion_coefficents_android.at<double>(0, cols) = distortion_coefficents_temp[element_counter];
					element_counter++;
				}


			have_calibration_values_android_dc = true;
			log_readJson << "Set distortion coeffients (Android): " << distortion_coefficents_android << endl;

		}
		else
			log_readJsonErr << "no information about previously done camera calibration [distortion_coefficents] available" << std::endl;


		if (j["calibration_rmse"] != nullptr) {
			rmse_calibration_android = std::stod(j.at("calibration_rmse").get<std::string>());
			have_calibration_values_android_rmse = true;
			log_readJson << "Set rmse camera calibration (Android): " << rmse_calibration_android << endl;

		}
		else
			log_readJsonErr << "no information about previously done camera calibration [calibration_rmse] available" << std::endl;



		// ask for previous determined exterior orientation
		// check if both information, rvec and tvec is avaiable, otherwise do nothing	
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

			// convert to opencv array
			rvecs_prev = cv::Mat::zeros(3, 1, CV_64FC1);
			tvecs_prev = cv::Mat::zeros(3, 1, CV_64FC1);

			int element_counter = 0;
			// Make a double loop over indexes and assign values
			for (int rows = 0; rows < 3; rows++) {
					rvecs_prev.at<double>(rows, 0) = rvec_prev_temp[element_counter];
					tvecs_prev.at<double>(rows, 0) = tvec_prev_temp[element_counter];
					element_counter++;
			}
			log_readJson << "Set exterior orientation - rvec: " << rvecs_prev << endl;
			log_readJson << "Set exterior orientation - tvec: " << tvecs_prev << endl;
			
			have_exterior_information = true;

		}
		
		



		// set projection center + height of hand-held camera position
		if (j["location_UTM_easting"] != nullptr && j["location_UTM_northing"] != nullptr && j["height"] != nullptr) {
		
			x0 = std::stod(j.at("location_UTM_easting").get<std::string>());
			y0 = std::stod(j.at("location_UTM_northing").get<std::string>());
			z0 = std::stod(j.at("height").get<std::string>()); // TODO abfangen von höhe!

			// define shifter for point cloud shifting!
			shifter_x = x0;
			shifter_y = y0;

			x0 -= shifter_x;
			y0 -= shifter_y;
			z0 += 1.50; // add height camera held by human person, average 1.50
			
			boundingBox->set_X0_Cam_World(x0, y0, z0);
			
			log_readJson << "set shifter_x, shifter_y for point cloud translation: " << std::fixed << shifter_x << ", " << shifter_y << endl;
			log_readJson << "set projection center (android location, shift to local cs, z += 1.50m): " << x0 << ", " << y0 << ", " << z0 << " & for bounding box" << endl;

			
		} 
		else
			log_readJsonErr << "no information about projection center available" << std::endl;


		// set orientation by angles
		if (j["azimuth"] != nullptr && j["pitch"] != nullptr && j["roll"] != nullptr) {

			azimuth = std::stof(j.at("azimuth").get<std::string>());
			pitch = std::stof(j.at("pitch").get<std::string>());
			roll = std::stof(j.at("roll").get<std::string>());
			
			boundingBox->setAngles(azimuth, roll, pitch);
			
			log_readJson << "set euler angles. (android sensor fusion): azimuth: " << azimuth << ", pitch: " << pitch << ", roll: " << roll << " & for bounding box" << endl;
		} 
		else
			log_readJsonErr << "no information about orientation angles available" << std::endl;

	
		// set pix size /--> use size of smart phone sensor (or hard definition to 0.001f px)
		if (j["pixel_size_mm_mean"] != nullptr) {
			pixSize = std::stof(j.at("pixel_size_mm_mean").get<std::string>()); 
			// Test!
			// pixSize = 0.00089f;
			log_readJson << "Set pixSize: " << pixSize << endl;
		} 
		else {
			log_readJsonErr << "No information about pixel size available" << std::endl;
		}

		
		// set dh = r. use resolution of google or android.location accuracy
		if (j["loc_accuracy"] != nullptr) {
			dh = std::stof(j.at("loc_accuracy").get<std::string>());
			r = dh;	
			boundingBox->set_r(r);
			boundingBox->set_dh(dh);
			log_readJson << "Set noise position location (r): " << r << " & for bounding box" << endl;
			log_readJson << "Set noise position height (h): " << dh << " & for bounding box" << endl;
			
		
			
		}
		else
			log_readJsonErr << "No information about location accuracy size available" << std::endl;


		// set focal length in mm
		if (j["focal_length_mm"] != nullptr) {
			ck = std::stof(j.at("focal_length_mm").get<std::string>());
			log_readJson << "Set focal length ck: " << ck << "mm" << endl;
		}
		else
			log_readJsonErr  << "No information about focal length of camera available" << std::endl;

		
		// view_angle_x, view_angle_y
		if (j["view_angle_x"] != nullptr && j["view_angle_y"] != nullptr) {
			H = std::stof(j.at("view_angle_x").get<std::string>());
			V = std::stof(j.at("view_angle_y").get<std::string>());

			// View Angle in Android auf kompletten Öffnungswinkel bezogen! Richard nutzt Angabe des halben Winkels für Berechnung!
			H /= 2;
			V /= 2;

			// Prüfe ob ViewWinkel zu groß ist! über 60 Grad macht keinen Sinn! --> Setze in diesem Fall auf 59.0 Grad
			if (H > 30.0f)
				H = 30.0f;
			if (V > 30.0f)
				V = 30.0f;
			boundingBox->setViewAngle(H, V);
			
			log_readJson << "Set opening angles view_x/view_y (H/V): " << H << "/" << V << " & for bounding box" << endl;	
		}
		else
			log_readJsonErr << "No information about view angle horizontal and/or vertical of camera available" << std::endl;

		
		// prüfe ob rotationsmatrix von android vorhanden ist. 
		if (j["rotationMatrix"] != nullptr) {
			std::string rotM = j.at("rotationMatrix").get<std::string>();

			int i = 0;
			while (i < rotM.size())	{
				if (rotM[i] == '[' || rotM[i] == ']'){
					rotM.erase(i, 1);
				} else {
					i++;
				}
			}
	
			std::stringstream ss(rotM);
		
			i = -1;
			while (ss.good() ) {
				i++;
				std::string substr;
				getline(ss, substr, ',');
			
				Rxyz[i] = std::stof(substr);
			}
			log_readJson << "Set rotation matrix (Android), Size: " << i+1 << endl;
		}
		else {
			log_readJsonErr << "No information about android rotation matrix available. Call bounding box for calculation." << std::endl;
			boundingBox->calc_rotationMatrix_xyz(Rxyz); // modification of rotation matrix
		}

		// set depth of point cloud. for real not used because of hard definition d = 200 m.
		if (j["distance"] != nullptr) {
			d = std::stof(j.at("distance").get<std::string>());
			boundingBox->setDist(d);
			log_readJson << "Set max depth point cloud (dist): " << d << " & for bounding box" << endl;
		}
		else {
			
			d = 200.0;
			boundingBox->setDist(d);
			
			log_readJsonErr << "No information about depth of point cloud available. use default value d = " << d << "[m]" << std::endl;
			log_readJson << "Set max depth point cloud (dist): " << d << " & for bounding box" << endl;

		}



		// set tolerance value to project points just in a defined area away from the user to avoid e.g. railings or others to be reprojected in the image
		if (j["tolerance_depth"] != nullptr) {
			thresh_for_point_projection = std::stof(j.at("tolerance_depth").get<std::string>());	
			log_readJson << "Set toleranceForPointProjection depth point cloud (dist): " << thresh_for_point_projection << " & for bounding box" << endl;
		}
		else {
			thresh_for_point_projection = 1.0f; // 10.0f;
			log_readJsonErr << "No information tolerance depth available. use default value thresh_closest_point = " << thresh_for_point_projection << "[m]" << std::endl;
			log_readJson << "Set max depth point cloud (dist): " << d << " & for bounding box" << endl;
		}

		//when finished with data input streaming, recalculate bounding box
		boundingBox->calcBoundingBox();
		
		// check if android calibration is valid (all bools have to be true)
		if (have_calibration_values_android_cm && have_calibration_values_android_dc && have_calibration_values_android_rmse)
			have_calibration_values_android = true;
		log_readJson << "have valid camera calibration [camera_matrix, distortion_coefficents, rmse] of client's camera" << endl;

		// print content of received json data
		printLogfile_log_readJson();

	}
	// --------------------------------------------- //










	


	// ----------------- IMAGE MANAGEMENT ---------------- //
	void setSynthImage(cv::Mat& _synthImage) { 
		_synthImage.copyTo(synthImage);
	}

	void setRealImage(cv::Mat& _realImage) {
		_realImage.copyTo(realImage);
	}

	void setCoordinateImage(int column, int row) {
		coord_Img = new CoordinateImage(column, row);
	}

	// set vegetation masks
	void setMaskTGIVegetationRealImage(cv::Mat mask) {
		mask.copyTo(maskTGI_Vegetation_real);
	}

	void setMaskTGIVegetationSynthImage(cv::Mat mask) { 
		mask.copyTo(maskTGI_Vegetation_synth); 
	}

	// get information about synth, real and RB´s coordinate image (own data type)
	cv::Mat& get_synth_image() {	return synthImage; }
	cv::Mat& get_real_image() {	return realImage; }
	CoordinateImage* getCoordinateImage() { return coord_Img; }
	
	// get information about vegetation masks
	cv::Mat get_mask_real_image_veg_TGI() { return maskTGI_Vegetation_real; }
	cv::Mat get_mask_synth_image_veg_TGI() { return maskTGI_Vegetation_synth; }

	bool haveVegetationMask_Real() {
		if (maskTGI_Vegetation_real.empty())
			return false;
		else
			return true;
	}

	bool haveVegetationMask_Synth() {
		if (maskTGI_Vegetation_synth.empty())
			return false;
		else
			return true;
	}
	// --------------------------------------------- //





	// ---------- POINT CLOUD MANAGEMENT --------- //

	// geben nur Zeiger auf das Element zurück
	std::vector<Vek2d>* get_synth_pts_2D() { return _synth_pts_2D_double; }
	std::vector<Vek3d>* get_synth_pts_3D() { return _synth_pts_3D_double; }
	std::vector<Vek3i>* get_pts_farben() { return _pts_farbe; }

	std::vector<Recolored_Point_Cloud>* get_point_cloud_recolored_ptr() { return punktwolke_neu_eingefaerbt; }
	
	// für bspw. UTM Shifting!
	double getShifter_x() { return shifter_x; }
	double getShifter_y() { return shifter_y; }
	void setShifter_x(double _shifter_x) { shifter_x = _shifter_x; }
	void setShifter_y(double _shifter_y) { shifter_y = _shifter_y; }

	// management of bounding box
	BoundingBox* getBoundingBox() { return boundingBox; }














	// --------------------------------------------- //


	bool generate_Batch_D2Net(const cv::Mat& trueImage, const cv::Mat& synthImage) {
		
		// check true and synth image to match
		if (trueImage.empty() || synthImage.empty()) {
			log_generateBatchFile << "cannot generate batch file: missing images" << std::endl;
			return false;
		} else {
			log_generateBatchFile << "generate batch file to run VisualSFM" << std::endl;
		}
	
		// set local parameters max_edge and max_sum_edges
		// caution! definition from d2net, do better not change (otherwise it will require a lot of VRAM on GPU or the calculation will take a lot of time!)
		const uint max_edge = 1600;
		const uint max_sum_edges = 2800;
		
		//define paths for in- and output 
		std::string path_directory_myData = (this->get_path_working_directory() + "\\myData\\");	
		CreateDirectoryA(LPCSTR(path_directory_myData.c_str()), NULL); //generate myData directory
		std::string path_file_imagelist_to_match = (path_directory_myData + "image_file_list.txt").c_str(); // generate imageList.txt & bat-file for d2Net
		std::string path_file_trueImage_to_match = (path_directory_myData + "1.jpg").c_str();
		std::string path_file_synthImage_to_match = (path_directory_myData + "2.jpg").c_str();
		path_file_output_D2Net_matches = (path_directory_myData + "kpts.txt").c_str();

		log_generateBatchFile << "path image_file_list.txt: " << path_file_imagelist_to_match << endl;
	
		// check if max_edge and max_sum_edges of trueImage and synthImage fit the requirements from d2net
		// otherwise, reduce image size (half sized currently)

		// calculate scaling factor
		d2Net_scalingFactor_trueImage = static_cast<double>(max_sum_edges) / (trueImage.size().width + trueImage.size().height); //0.5; 
		d2Net_scalingFactor_synthImage = static_cast<double>(max_sum_edges) / (synthImage.size().width + synthImage.size().height); //0.5
		std::cout << "d2net: scaling factors, true image: " << d2Net_scalingFactor_trueImage << ", synth image: " << d2Net_scalingFactor_synthImage << endl;
		log_generateBatchFile << "d2net: scaling factors, true image: " << d2Net_scalingFactor_trueImage << ", synth image: " << d2Net_scalingFactor_synthImage << endl;

		// apply scaling factor
		cv::Mat trueImage_scaled, synthImage_scaled;
		cv::resize(trueImage, trueImage_scaled, cv::Size(), d2Net_scalingFactor_trueImage, d2Net_scalingFactor_trueImage);
		cv::resize(synthImage, synthImage_scaled, cv::Size(), d2Net_scalingFactor_synthImage, d2Net_scalingFactor_synthImage);
		cv::imwrite(path_file_trueImage_to_match, trueImage_scaled);
		cv::imwrite(path_file_synthImage_to_match, synthImage_scaled);

		

		//prüfe ob bilder bereits gespeichert wurden und pfad somit aktiv
		while (calculateFileSize(path_file_trueImage_to_match) == NULL || calculateFileSize(path_file_synthImage_to_match) == NULL || calculateFileSize(path_file_trueImage_to_match) < 1 || calculateFileSize(path_file_synthImage_to_match) < 1) {
			// mache nix solang kein Bild da ist!
		}

		log_generateBatchFile << "file size real image: " << calculateFileSize(path_file_trueImage_to_match) << ", file size virtual image: " << calculateFileSize(path_file_synthImage_to_match) << endl;

		// lege Filestream für image_list_file und BatchFile an
		std::ofstream myFileImageList;
		myFileImageList.open(path_file_imagelist_to_match); // Ablage im WD
		myFileImageList << path_file_trueImage_to_match << endl;
		myFileImageList << path_file_synthImage_to_match << endl;

		myFileImageList.close();

		while (calculateFileSize(path_file_imagelist_to_match) == NULL) {
		}

		// erzeuge BatchFile
		path_file_batch_call_pyD2Net = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\d2net\\myBatchD2Net-1.bat").c_str();
		std::string path_d2net = (path_directory_feature_matching_tool.substr(0, path_directory_feature_matching_tool.find_last_of("\\/")) + "\\d2net").c_str();

		std::ofstream myD2NetBatchFile;
		myD2NetBatchFile.open(path_file_batch_call_pyD2Net); // Ablage im WD
		myD2NetBatchFile 
			<< "cd " << path_d2net
			<< " &" 
			<< " %windir%\\System32\\cmd.exe /k" 
		 	<< " \"\"C:\\ProgramData\\Anaconda3\\Scripts\\activate.bat\" env_d2net & python --version & python.exe " << path_d2net << "\\extract_features.py" 
			<< " --image_list_file " << path_file_imagelist_to_match 
			<< " --max_edge " << max_edge 
			<< " --max_sum_edges " << max_sum_edges
			<< " & exit() & cd " << path_directory_myData << "\"" << endl;
		myD2NetBatchFile.close(); // Ablage im WD
	
		return true;
	}





	// ---------- GENERATE BATCH FILE 4 VISUAL SFM --------- //
	// defines path to generated batch file
	// input: real and synth image (filtered or origninal)
	// return: true/false in case of success/non success
	bool generate_Batch_VisualSfM(const cv::Mat& _realImage, const cv::Mat& _synthImage) {
		
		//Dateipfad
		std::string wd_batch = (this->get_path_working_directory() + "\\myData\\");
		CreateDirectoryA(LPCSTR(wd_batch.c_str()), NULL);

		// erzeuge hier imageList.txt und Batchfile für weiterarbeit mit VSFM
		std::string pathImageList = (wd_batch + "myImageList.txt").c_str();
		std::string pathRealImage = (wd_batch + "realImage.jpg").c_str();
		std::string pathSynthImage = (wd_batch + "synthImage.jpg").c_str();

		log_generateBatchFile << "path myImageList.txt: " << pathImageList << endl;

		if (_realImage.empty() || _synthImage.empty()) {
			log_generateBatchFile << "cannot generate batch file: missing images" << std::endl;
			return false;
		}
		else {
			log_generateBatchFile << "generate batch file to run VisualSFM" << std::endl;
		}	
			
		// speichere Real/SynthBild im WD ab
		cv::imwrite(pathRealImage, _realImage);
		cv::imwrite(pathSynthImage, _synthImage);
		
		//prüfe ob bilder bereits gespeichert wurden und pfad somit aktiv
		while (calculateFileSize(pathRealImage) == NULL || calculateFileSize(pathSynthImage) == NULL || calculateFileSize(pathRealImage) < 1 || calculateFileSize(pathSynthImage) < 1) {
			// mache nix solang kein Bild da ist!
		}

		log_generateBatchFile << "file size real image: " << calculateFileSize(pathRealImage) << ", file size virtual image: " << calculateFileSize(pathSynthImage) << endl;



		// lege Filestream für ImageList und BatchFile an
		std::ofstream myFileImageList;
		
		myFileImageList.open(pathImageList); // Ablage im WD
		myFileImageList << pathRealImage << endl;
		myFileImageList << pathSynthImage << endl;
	
		myFileImageList.close();

		while (calculateFileSize(pathImageList) == NULL) { 
		}

		// erzeuge BatchFile
		path_file_batch_call_exeVSfM = (wd_batch + "myBatchSfM.bat").c_str();
		path_file_output_VSfM_matches = (wd_batch + "outputSfmMatches.txt").c_str();

		// Vsfm --> hole Pfad neu aus Verzeichnis!
		//char buff[FILENAME_MAX];
		//GetCurrentDir(buff, FILENAME_MAX);
		
		std::string tempPath = path_directory_feature_matching_tool;
		log_generateBatchFile << tempPath.substr(0, tempPath.find_last_of("\\/")) << endl;
		
		std::string pathVsfm = (tempPath.substr(0, tempPath.find_last_of("\\/")) + "\\VisualSFM\\VisualSFM.exe").c_str();


		std::ofstream myVsfmBatchFile;
		myVsfmBatchFile.open(path_file_batch_call_exeVSfM); // Ablage im WD


		myVsfmBatchFile << "@echo off" << endl;
		myVsfmBatchFile << pathVsfm << " sfm " << pathImageList << " " << path_file_output_VSfM_matches << endl;
		myVsfmBatchFile << pathVsfm << " sfm[pairs+exportf] " << pathImageList << " " << path_file_output_VSfM_matches << endl;

		myVsfmBatchFile.close(); // Ablage im WD

		return true;
	}










	std::string getPathOutputFile_Vsfm() { return path_file_output_VSfM_matches; }
	std::string getPathOutputFile_D2Net() { return path_file_output_D2Net_matches; }
	std::string getPathBatchFile_VSfM() { return path_file_batch_call_exeVSfM; }
	std::string getPathBatchFile_D2Net() { return path_file_batch_call_pyD2Net; }
	std::string getExeDirectory() { return path_directory_feature_matching_tool; }
	void setDirectoryExecutable(std::string value) { path_directory_feature_matching_tool = value; }
	std::string getPathBatchFile_EllipsoidJar() { return path_file_batch_call_jarEllipsoid; }
	void setPath_to_ellipsoid_jar_batch(std::string _path) { path_file_batch_call_jarEllipsoid = _path; }


	// --------------------------------------------- //





	
	
	// needs #include <fstream>
	int calculateFileSize(std::string path) {
		FILE *pFile = NULL;	
		fopen_s(&pFile, path.c_str(), "rb"); // get the file stream
		fseek(pFile, 0, SEEK_END); // set the file pointer to end of file
		int Size = ftell(pFile); // get the file size
		// rewind( pFile ); // return the file pointer to begin of file if you want to read it		
		fclose(pFile); // close stream and release buffer

		return Size;
	}

	
	// calculate BoundingBox
	void calculateBoundingBox() {
		boundingBox->calcBoundingBox();
	}







	// ------ printers ------
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



	// ----------- getters / setters ---------------------

	// PATHS
	// get/set path to working directory
	void set_path_working_directory(std::string path_dir) {
		path_working_directory = path_dir;
		logFilePrinter->append(TAG + "set working directory : " + path_working_directory);
	}
	std::string get_path_working_directory() { return path_working_directory; }

	// get/set path to result directory
	void set_path_directory_result(std::string path_dir) {
		path_directory_result = path_dir;
		logFilePrinter->append(TAG + "set result directory : " + path_directory_result);
	}
	std::string get_path_directory_result() { return path_directory_result; }
	
	// get/set path of point cloud used for synthetic image rendering (pointcloud.pw file)
	void set_path_file_pointcloud(std::string path) {
		path_file_point_cloud = path;
		log_readJson << "Set input point cloud " << path << endl;
	}
	std::string get_path_file_pointcloud() { return path_file_point_cloud; }

	

	// get file name of true (master) image (set during read out of JSON meta data file)
	std::string  get_filename_true_image() { return file_name_true_image; }
	
	// get image size of true image
	cv::Size get_size_true_image() { return imageSize; }
	
	


	// get logfile
	LogFile* getLogFilePrinter() { return logFilePrinter; }

	// get information about water line
	std::vector<cv::Point2d>* get_water_line_image_points_2D_ptr() { return waterlinePoints; }
	bool& get_have_water_line_image_points_2D_ptr() { return have_water_line_image_points_2D_ptr; }

	// get camera attributes
	float& get_pixel_size() { return pixSize; }
	float& getOpeningAngleHorizontal() { return H; }
	float& getOpeningAngleVertical() { return V; }
	float& getFocalLength() { return ck; }
	
	// in case of available camera information from android
	bool get_have_camera_calibration_android_cm() { return have_calibration_values_android_cm; } // cam matrix
	bool get_have_camera_calibration_android_dc() { return have_calibration_values_android_dc; } // distortion coefficients
	
	cv::Mat get_camera_calibration_android_cm() { 
		std::stringstream cam_mat_string; cam_mat_string << camera_matrix_android;
		logFilePrinter->append(TAG + "Deliver camera matrix: " + cam_mat_string.str());
		return camera_matrix_android; }
	
	cv::Mat get_camera_calibration_android_dc() { 
		std::stringstream dist_mat_string; dist_mat_string << distortion_coefficents_android;
		logFilePrinter->append(TAG + "Deliver distortion coefficents: " + dist_mat_string.str());
		return distortion_coefficents_android; }
	
	double get_camera_calibration_android_rmse() { return rmse_calibration_android; }

	

	

	// ----- MetaData 	 ------
	std::string& get_uuid() { return uuid; } 	// get uuid
	
	// get information about projection settings
	float& getDistance() { return d; }
	float& getDistanceNoise() { return dh; }
	float& getWidenessNoise() { return r; }
	float& getThresholdForPointProjection() { return thresh_for_point_projection; }
	

	// TRANSLATION / ROTATION PARAMETERS
	bool get_have_exterior_information() { return have_exterior_information; }	// for extrinsic information being available

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

	// get projection center
	double& getX0() { return x0; }
	double& getY0() { return y0; }
	double& getZ0() { return z0; }
	cv::Point3d& getProjectionCenter() { return cv::Point3d(x0, y0, z0); }

	// set projection center
	void setX0(double _x0) { x0 = _x0; };
	void setY0(double _y0) { y0 = _y0; };
	void setZ0(double _z0) { z0 = _z0; };
	void setProjectionCenter(double _x0, double _y0, double _z0) {
		x0 = _x0;
		y0 = _y0;
		z0 = _z0;
	}

	// set projection center and define shifters new!
	// shifters are necessary to work with large coordinates (e.g. UTM coordinates) --> needed to reduce the lengths of coordinates to work with
	void setProjectCenter_applyShifter(double _x0, double _y0, double _z0) {
		// define shifter for point cloud shifting!
		shifter_x = _x0;
		shifter_y = _y0;

		x0 = _x0 - shifter_x;
		y0 = _y0 - shifter_y;
		z0 = _z0;

		boundingBox->set_X0_Cam_World(x0, y0, z0);
	}

	// get orientation angles
	float& getAzimuth() { return azimuth; }
	float& getRoll() { return roll; }
	float& getPitch() { return pitch; }

	// get rotation matrix
	float* getRotationMatrix() { return Rxyz; } 

	// set rotation matrix
	void setRotationMatrix_cvMat(cv::Mat _rotM) {
		
		Rxyz[0] = _rotM.at<double>(0,0);
		Rxyz[1] = _rotM.at<double>(1,0);
		Rxyz[2] = _rotM.at<double>(2,0);
		Rxyz[3] = _rotM.at<double>(0,1);
		Rxyz[4] = _rotM.at<double>(1,1);
		Rxyz[5] = _rotM.at<double>(2,1);
		Rxyz[6] = _rotM.at<double>(0,2);
		Rxyz[7] = _rotM.at<double>(1,2);
		Rxyz[8] = _rotM.at<double>(2,2);		

	}

	void setRz_4_BBox(cv::Mat _rotM) {
		
		Rz[0] = _rotM.at<double>(0, 0);
		Rz[1] = _rotM.at<double>(0, 1);
		Rz[2] = 0;
		Rz[3] = -_rotM.at<double>(0, 1);
		Rz[4] = _rotM.at<double>(0, 0);
		Rz[5] = 0;
		Rz[6] = 0;
		Rz[7] = 0;
		Rz[8] = 1;

		boundingBox->set_Rz(Rz);
	}



	// ----- Matching 	 ------
	// in case of failed matching, give error message to main and exit program safely (call destructors, ...)
	void setFailedMatching(bool value) { failedMatching = value; } // defined in Matching.cpp
	bool getFailedMatching() { return failedMatching; }

	// get d2Net scaling factor for true and synthetic image to match max_edge_sum and max_edge
	double get_d2Net_scalingFactor_trueImage() { return d2Net_scalingFactor_trueImage; }
	double get_d2Net_scalingFactor_synthImage() { return d2Net_scalingFactor_synthImage; }

	// get/set infos about object point distribution & IO refinement (if or if not!)
	void set_well_distributed_object_points_3D_space(bool val) { well_distributed_object_points_3D_space = val; }
	bool get_well_distributed_object_points_3D_space() { return well_distributed_object_points_3D_space; }
	void set_well_distributed_object_points_image_space(bool val) { well_distributed_object_points_image_space = val; }
	bool get_well_distributed_object_points_image_space() { return well_distributed_object_points_image_space; }





private:

	const std::string TAG = "DataManager:\t";


	// paths to directories or files
	std::string path_working_directory = "noDir";
	std::string path_directory_feature_matching_tool = "noDir";
	std::string path_directory_result = "noDir";
	std::string path_file_output_VSfM_matches = "noDir";
	std::string path_file_output_D2Net_matches = "noDir";
	std::string path_file_batch_call_jarEllipsoid = "noDir";
	std::string path_file_batch_call_exeVSfM = "noDir";
	std::string path_file_batch_call_pyD2Net = "noDir";
	std::string file_name_true_image = "noFn";

	// logoutput (gesammelt plotten)
	std::stringstream log_readJson, log_readJsonErr; 
	std::stringstream log_generateBatchFile;
	
	// init variables with default values
	std::string path_file_point_cloud, output;
	double x0, y0, z0;
	cv::Size imageSize; 
	cv::Mat realImage; std::string pathToMasterImage;
	cv::Mat synthImage;

	// uuid client from json
	std::string uuid = "no_uuid";

	// define bounding box for point cloud extent
	BoundingBox* boundingBox; 
	
	float azimuth, roll, pitch, pixSize, d, dh, r, H, V, ck;
	
	bool have_water_line_image_points_2D_ptr;

	// Punkte für späteres Matching
	std::vector<Vek2d>* _synth_pts_2D_double;
	std::vector<Vek3d>* _synth_pts_3D_double;
	std::vector<Vek3i>* _pts_farbe;
	
	std::vector<cv::Point2d>* waterlinePoints;
	//std::vector<cv::Point3d>* punktwolke;
	//std::vector<cv::Point3d>* punktwolkeFiltered;
	std::vector<Recolored_Point_Cloud>* punktwolke_neu_eingefaerbt;

	CoordinateImage* coord_Img;

	// Maske für Vegetation im Bild aus TGI Daten
	cv::Mat maskTGI_Vegetation_real, maskTGI_Vegetation_synth;
	bool haveVegetationMask = false;



	double shifter_x = 0.0, shifter_y = 0.0; // erlaube Shifting von großen Punktwolken für höhere Genauigkeit und leichters Handling beim matching!
	
	bool failedMatching = false;

	
	// schreibe hier korrigierte Werte aus solvepnp und co rein!
	cv::Point3d projCenter_Corr;
	cv::Vec3f eulerAngles_Corr;
	double focal_length_Corr;
	cv::Point2d principle_point_Corr;

	//std::vector<float> R_android; 
	float* Rxyz;
	float* Rz;

	LogFile* logFilePrinter;
	
	cv::Mat camera_matrix_android = cv::Mat(3, 3, CV_64FC1); 
	cv::Mat distortion_coefficents_android = cv::Mat(1, 5, CV_64FC1);

	double rmse_calibration_android;
	bool have_calibration_values_android_cm, have_calibration_values_android_dc, have_calibration_values_android_rmse, have_calibration_values_android = false;

	// in case of previous done exterior orientation determinatin
	cv::Mat tvecs_prev, rvecs_prev; 
	bool have_exterior_information = false;

	// tolerance threshold value for depth definition to avoid e.g. railings etc. being projected inside the image
	// Prüfe ob der zu projizierende Punkt zu nah am Projektionszentrum liegt, wenn loc_accuracy vergeben wurde. 1m standardmäßig abstand halten!
	float thresh_for_point_projection;
	

	// check if distribution of object points is sufficient to refine cameras intrinsics
	bool well_distributed_object_points_3D_space = false;  
	bool well_distributed_object_points_image_space = false;  // well distributed means: in each quadric of image are matched image points
	// well distributed means: in each quadric of image are matched image points
	// ellipsoid arround matched object points has appropriate Eigenvalues [not NAN --> then, Ellipsoid has only two or less dimensions --> not good for IO refinement!]
	double d2Net_scalingFactor_trueImage = 1.0;
	double d2Net_scalingFactor_synthImage = 1.0;
};


