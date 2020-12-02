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
#include "SpectralIndices.h"
#include <windows.h>
#include <ctime>
#include "LogfilePrinter.h"
#include <filesystem>
#include "boost/filesystem.hpp"
#include "Utils.h"


//#ifdef _WIN64
//#include <direct.h>
//#define GetCurrentDir _getcwd
//#elif __linux__
//#include <unistd.h>
//#define GetCurrentDir getcwd
//#endif

const::std::string NOTAG = "";
const::std::string TAG = "Main:\t\t";
LogFile* logFilePrinter;
DataManager* dataManager;
Matching* matching;	
PerspectiveImage* pim;

// get path of dirctory where the tool is running
std::string GetCurrentWorkingDir()
{
	char buff[FILENAME_MAX];
	GetCurrentDir(buff, FILENAME_MAX);
	std::string current_working_dir(buff);
	return current_working_dir;
}

int terminationProgram(PerspectiveImage* pim, DataManager* dataManager) {
	if (pim != nullptr)
		delete pim;
	if (dataManager != nullptr)
		delete dataManager;

	return (EXIT_FAILURE);
}

/**
arguments: 1) point_cloud.pw 2) json_file.json
**/
int main(int argc, char** argv)
{
	// First things first: check if necessary arguments are valid, else return before do anything!
	if (argc < 2) {
		logFilePrinter->append(TAG + " need more arguments!\n - i filePathPointCloud\\poincloud.pw - j filePathJson\\jsonFile.json");
		terminationProgram(nullptr, nullptr);
	}

	// -----
	// FLAGS
	// -----
	int approach = DataManager::D2NET; //Flags_Matching_Approach::VSFM;
	  
	float neighbourDistance_allowed_d2net = 0.25f; // allowed neighbouring distance for knn search
	float neighbourDistance_allowed_vsfm = 0.99f; // <- previously used value but the smaller the better. Might only work with d2net. 


	// ----
	// INIT
	// ----
	logFilePrinter = new LogFile();
	dataManager = new DataManager(logFilePrinter);	// 2. init dataManager for data storage and managment // using like interface
	

	logFilePrinter->append(NOTAG + 
		"-----------------------------" + "\n" +
		"START VIRTUAL-IMAGE-GENERATOR" + "\n" +
		"-----------------------------");
	logFilePrinter->append(TAG + "initialised 'logFilePrinter' and 'dataManager'");


	// ------------------
	// SET UP DIRECTORIES
	// ------------------
	// create results directory (used for all results of all requests)
	std::string path_directory_results = (GetCurrentWorkingDir() + "\\" + "Result" + "\\");
	CreateDirectoryA(LPCSTR(path_directory_results.c_str()), NULL); // erstelle Verzeichnis
	dataManager->set_path_directory_result(path_directory_results);
	logFilePrinter->append(TAG + "directory of results: " + path_directory_results);

	//create working directory depending on processing date/time --> dir structure: results\wd_localtime (no UTC! if UTC is desired use 'tm *gmtm = gmtime(&now); dt = asctime(gmtm);'
	char localTime[20];
	time_t now = time(0);
	strftime(localTime, 20, "%Y-%m-%d_%H-%M-%S", localtime(&now)); // current date/time based on current system
	
	std::string path_working_directory = path_directory_results + localTime + "\\";
	CreateDirectoryA(LPCSTR(path_working_directory.c_str()), NULL); // generate directory for current processing
	dataManager->set_path_working_directory(path_working_directory); // set working directory in dataManager
	logFilePrinter->append(TAG + "working directory: " + path_working_directory);
	
	// set directory of executable (necessary for VSFM/D2Net)
	dataManager->setDirectoryExecutable(argv[0]);
	logFilePrinter->append(TAG + "directory of feature matching script: " + std::string(argv[0]));


	// -------------------
	// READ ARGUMENTS ARGV 
	// -------------------

	std::string path_file_pointcloudPW, path_file_jsonTxt;

	// interpret input arguments from cmd line
	std::stringstream s;
	for (int i = 1; i < argc; ++i) {

		char check = argv[i][1];
		switch (check) {

		case 'i': path_file_pointcloudPW = argv[i + 1];
			if (argv[i + 1][0] == '"') {
				int na = path_file_pointcloudPW.find_first_of("\"");
				int nb = path_file_pointcloudPW.find_last_of("\"");
				path_file_pointcloudPW = path_file_pointcloudPW.substr(na + 1, nb - na - 1);
			}

			logFilePrinter->append(TAG + "Read ARGV. Set path to point cloud (PW-file): " + path_file_pointcloudPW);
			dataManager->set_path_file_pointcloud(path_file_pointcloudPW); // set path point cloud (input)

			i += 1;
			break;

		case 'j': path_file_jsonTxt = argv[i + 1];
			if (argv[i + 1][0] == '"') {
				int na = path_file_jsonTxt.find_first_of("\"");
				int nb = path_file_jsonTxt.find_last_of("\"");
				path_file_jsonTxt = path_file_jsonTxt.substr(na + 1, nb - na - 1); // set path of JSON file (jsonTxt)
			}
			
			logFilePrinter->append(TAG + "Read ARGV. Set path to json (TXT-file): " + path_file_jsonTxt);
			dataManager->read_json_file(path_file_jsonTxt); // 5. set and read json. using dataManager for reading and storage 
			logFilePrinter->append(TAG + "Read json file");

			i += 1;
			break;
		}
	}



	

	// NEW: iterative approach, start with first i2g and generate new synth image, try to refine parameters, if repro similar, stop and transform water line into object space
	float global_repro_error = -1.0f;
	bool repeat_i2g = true; // needs to be true! if no iterations are considered use the iteration counter 
	int iteration = 0; // stdval: 2
	int const FINAL_ITERATION = 0; //in sum: 3 iterations

	cv::Mat camera_matrix, dist_coeffs, tvec, rvec;
	bool should_I_refine_IOP = false;
	
	while (repeat_i2g) {

		logFilePrinter->append(TAG + "------------------ ITERATION " + std::to_string(iteration) + "---------------------");

		// 6. generate virtual image (central perspective), store in dataManager 
		pim = new PerspectiveImage(dataManager);
		pim->generateImage();


		// ---- INFO ---- 6.a) and 6.b) are currently of
		// 6.a) generate mask of vegetation
		//std::string wd = dataManager->getWorkingDirectory() + "\\VegetationMask\\";
		//CreateDirectoryA(LPCSTR(wd.c_str()), NULL);

		//cv::Mat maskVeggiRealImage = TGI(logFilePrinter, dataManager->get_real_image(), wd + "TGI_real"); // Berechne hier TGI Bild und füge es direkt in DataManager ein!
		//dataManager->setMaskTGIVegetationRealImage(maskVeggiRealImage); //füge in DatenManager ein

		//cv::Mat maskVeggiSynthImage = TGI(logFilePrinter, dataManager->get_synth_image(), wd + "TGI_synth"); // Berechne hier TGI Bild und füge es direkt in DataManager ein!
		//dataManager->setMaskTGIVegetationSynthImage(maskVeggiSynthImage); //füge in DatenManager ein

		// 6.b) image filtering, currently implemented: Wallis, Gaussian, FastNlMeans_Color, Bilateral_Color
		// !!! only for first iteration regarding real image! otherwise, the pre-processing would be performed ever and ever again!
		if (iteration == 0) {
			dataManager->set_true_image(pim->applyFilterAlgorithms(dataManager->get_true_image(), DataManager::APPLY_GAUSSIAN)); // slight smoothing because of unsharpness of rendered virtual image
		    //dataManager->setRealImage(pim->applyFilterAlgorithms(dataManager->get_real_image(), PerspectiveImage::APPLY_WALLIS)); // slight smoothing because of unsharpness of rendered virtual image																													
		}
		dataManager->set_synth_image(pim->applyFilterAlgorithms(dataManager->get_synth_image(), DataManager::APPLY_GAUSSIAN)); // slight smoothing because of point pattern due to rendered virtual image
		//dataManager->setSynthImage(pim->applyFilterAlgorithms(dataManager->get_synth_image(), PerspectiveImage::APPLY_WALLIS)); // slight smoothing because of point pattern due to rendered virtual image

		
		// calcuate vegetation masks (copy! no overwriting!)
		//cv::Mat real_veggi_mask, synth_veggi_mask;
		//dataManager->get_real_image().copyTo(real_veggi_mask, maskVeggiRealImage);
		//dataManager->get_synth_image().copyTo(synth_veggi_mask, maskVeggiSynthImage);



		// 7. prepare image matching
		// use realToMatch and synthToMatch for matching procedure
		cv::Mat realToMatch, synthToMatch;

		// NO APPLY vegetation mask
		dataManager->get_true_image().copyTo(realToMatch);
		dataManager->get_synth_image().copyTo(synthToMatch);

		// APPLY vegetation mask
		// real_veggi_mask.copyTo(realToMatch); // mit Veggi /// dataManager->getRealImage(); // ohne Veggi
		// synth_veggi_mask.copyTo(synthToMatch); // mit Veggi /// dataManager->getSynthImage(); // Ohne Veggi



		// TODO: implement flag to choose VSFM or D2NET matching
		// 8. a) generate batch file for execution of Visual SFM for SIFT feature detection and image matching. 
		// wait until batch file is ready for execution of vsfm.exe
		while (!dataManager->generate_batch_matching(realToMatch, synthToMatch, approach)) {
			logFilePrinter->append(TAG + "wait for generation batch file for image matching .");
		};
		dataManager->printLogfile_log_generateBatchFile(); // print log

		//8.b) generate bathc file for execution of D2Net for Feature Detection & Image Matching
		// wait until batch file is ready for execution
		//while (!dataManager->generate_Batch_D2Net(realToMatch, synthToMatch)) {
		//	logFilePrinter->append(TAG + "wait for generation batch file: D2Net.");
		//};


		// check which parameters are available for interior and exterior orientation
		// init IO + distC
		double focal_length_px = dataManager->getFocalLength() / dataManager->get_pixel_size();
		cv::Point2d center = cv::Point2d(dataManager->get_true_image().cols / 2, dataManager->get_true_image().rows / 2); // calc image center for initial principle point				
		camera_matrix = cv::Mat(3, 3, CV_64FC1);
		camera_matrix = (cv::Mat_<double>(3, 3) << focal_length_px, 0, center.x, 0, focal_length_px, center.y, 0, 0, 1);
		dist_coeffs = cv::Mat::zeros(5, 1, CV_64FC1);

		// init EO
		tvec = cv::Mat();
		tvec.push_back(dataManager->getProjectionCenter().x);
		tvec.push_back(dataManager->getProjectionCenter().y);
		tvec.push_back(dataManager->getProjectionCenter().z);


		rvec = cv::Mat();
		float* rotM_ptr = dataManager->getRotationMatrix();		// receive pointer rotation matrix
		for (int i = 0; i < 9; i++)
			rvec.push_back(rotM_ptr[i]);

		//init FLAGS
		Matching::Flags_resec flags_matching = Matching::FIX_NOTHING; //init

		// TODO! check distribution of object points, use IO optimisation only if distribution is sufficient 
		// first iteration, fix IO + distC and optimise EO only
		logFilePrinter->append(TAG + "Should refine IOP?: " + std::to_string(should_I_refine_IOP));
		if (iteration == 0) { // first it -> fix IO, EO only
			flags_matching = Matching::FIX_INTR;
			logFilePrinter->append(TAG + "Matching Flags: " + "FIX_INTR" + " (1st iteration) ");
		}
		else if (!should_I_refine_IOP) {
			flags_matching = Matching::FIX_INTR;  //refine IO if flag is false (IO refinement denied) otherwise IO fixed
			logFilePrinter->append(TAG + "Matching Flags: " + "FIX_INTR" + " (IOP refinement denied) ");
		} 
		else {
			flags_matching = Matching::FIX_NOTHING; // refine IO
			logFilePrinter->append(TAG + "Matching Flags: " + "FIX_NOTHING" + " (IOP refinement allowed) ");
		}
		logFilePrinter->append(TAG + "set flags_matching variable. checked extr parameters ");




		// 9. Initialisation of matching 
		// start image-2-geometry intersection
		matching = new Matching();
		matching->init(dataManager);

		// 10. spatial-resection in case of need (no fix of intr and extr parameters)
		if (flags_matching != Matching::FIX_INTR_EXTR) {

			// init vectors for resulting matches between synth and real  image
			std::vector<cv::Point3d> matched_object_points;
			std::vector<cv::Point2d> matched_image_points_real, matched_image_points_synth;

			// Option 1 : Execute visual sfm for sift-based matching 
			if (approach == DataManager::VSFM) {
				// try execution of vsfm batch file, otherwise throw error message
				// run batch script, print 'tool_name'
				logFilePrinter->append(NOTAG + 
					"-----------------------------------------" + "\n" +
					"START FEATURE DETECTION / MATCHING : VSFM" + "\n" +
					"-----------------------------------------");

				if (Utils::executeBatch(dataManager->getPathBatchFile_VSfM().c_str(), "Visual SFM")) {
					logFilePrinter->append(TAG + "processing by VisualSFM successful, read inliers passing fundamental test...");

					// wait until images are saved and path of visual sfm output becomes valid
					while (Utils::calculateFileSize(dataManager->getPathOutputFile_Vsfm()) == NULL || Utils::calculateFileSize(dataManager->getPathOutputFile_Vsfm()) < 1) {}

					// run post-processing of visual sfm inlier data
					// calculate interior and exterior camera parameters, necessary for transformation into object space
					matching->loadMatches(
						dataManager->getPathOutputFile_Vsfm().c_str(),
						dataManager->get_true_image(),
						dataManager->get_synth_image(),
						*dataManager->get_water_line_image_points_2D_ptr(),
						*dataManager->get_pts_synth_2D_double(),
						*dataManager->get_pts_synth_3D_double(),
						matched_object_points,
						matched_image_points_real,
						matched_image_points_synth,
						1.0,
						1.0,
						DataManager::VSFM,
						neighbourDistance_allowed_vsfm);

					// perform spatial resection using these points for camera and exterior orientation determination, check if values for camera are valid and available
					cv::Mat canvas_real_image = cv::Mat();
					dataManager->get_true_image().copyTo(canvas_real_image);
					float repro_error = matching->enhanced_spatial_resection(matched_object_points, matched_image_points_real, canvas_real_image, dataManager->get_pixel_size(), camera_matrix, dist_coeffs, rvec, tvec, flags_matching);

					logFilePrinter->append(TAG + "Reprojection Error: " + std::to_string(repro_error));


				}
				else {
					// in case of missing Visual SfM, terminate program
					logFilePrinter->append(TAG + "cannot continue, no visual sfm for sift based image matching available!");
				}
			}
			

			if (approach == DataManager::D2NET) {
				// Option 2 : Execute d2net for DL-guided image matching
				logFilePrinter->append(NOTAG +
					"-----------------------------------------" + "\n" +
					"START FEATURE DETECTION / MATCHING : D2NET" + "\n" +
					"-----------------------------------------");
				

				if (Utils::executeBatch(dataManager->getPathBatchFile_D2Net().c_str(), "D2 Net")) {
					logFilePrinter->append(TAG + "processing by D2Net successful, read inliers passing fundamental test...");


					// wait until images are saved and path of visual sfm output becomes valid
					while (Utils::calculateFileSize(dataManager->getPathOutputFile_D2Net()) == NULL || Utils::calculateFileSize(dataManager->getPathOutputFile_D2Net()) < 1) {

					}

					// run post-processing of visual sfm inlier data
					// calculate interior and exterior camera parameters, necessary for transformation into object space
					//matching->loadD2netMatches(
					matching->loadMatches(
						dataManager->getPathOutputFile_D2Net().c_str(),
						dataManager->get_true_image(),
						dataManager->get_synth_image(),
						*dataManager->get_water_line_image_points_2D_ptr(),
						*dataManager->get_pts_synth_2D_double(),
						*dataManager->get_pts_synth_3D_double(),
						matched_object_points,
						matched_image_points_real,
						matched_image_points_synth,
						dataManager->get_d2Net_scalingFactor_trueImage(),
						dataManager->get_d2Net_scalingFactor_synthImage(),
						DataManager::D2NET,
						neighbourDistance_allowed_d2net
					);


					// perform spatial resection using these points for camera and exterior orientation determination, check if values for camera are valid and available
					cv::Mat canvas_real_image = cv::Mat();
					dataManager->get_true_image().copyTo(canvas_real_image);
					float repro_error = matching->enhanced_spatial_resection(matched_object_points, matched_image_points_real, canvas_real_image, dataManager->get_pixel_size(), camera_matrix, dist_coeffs, rvec, tvec, flags_matching);

					logFilePrinter->append(TAG + "Reprojection Error: " + std::to_string(repro_error));

				}
				else {
					// in case of missing Visual SfM, terminate program
					logFilePrinter->append(TAG + "cannot continue, no visual sfm for sift based image matching available!");
					terminationProgram(pim, dataManager);
				}


				// check of matching results are suitable (more matches then 4), otherwise terminate program
				if (dataManager->getFailedMatching() == true) {
					logFilePrinter->append(TAG + "cannot continue, no matching results available!");
					terminationProgram(pim, dataManager);
				}
			}
	
		}



		// ACHTUNG IMAGE BLUR IMMER MEHR BEI REAL IMAGE!!!! GAUSS RAUS NEHMEN TODO!!!
		// Test
		// 3-vector to 9-rotM
		cv::Mat rotM;
		cv::Rodrigues(rvec, rotM);
		rvec = rotM.clone();

		dataManager->setRotationMatrix_cvMat(rotM);
		dataManager->setRz_4_BBox(rotM);


		// convert translation vector
		cv::Mat tvec_new, rvec_transp;
		cv::transpose(rvec, rvec_transp);
		tvec_new = -rvec_transp * tvec;

		// apply shifting of point cloud origin
		tvec_new.at<double>(0, 0) = tvec_new.at<double>(0, 0) + dataManager->get_utm_shift_x();
		tvec_new.at<double>(0, 1) = tvec_new.at<double>(0, 1) + dataManager->get_utm_shift_y();
		std::cout << TAG << "Tvec_new_shifted\n" << tvec_new << std::endl;
		logFilePrinter->append(TAG + "Tvec_new_shifted\n" + std::to_string(tvec_new.at<double>(0)) + "," + std::to_string(tvec_new.at<double>(1)) + ";" + std::to_string(tvec_new.at<double>(2)));
		
		dataManager->set_projC_zeroOrigin_updateShiftVals_setX0BBox(tvec_new.at<double>(0), tvec_new.at<double>(1), tvec_new.at<double>(2));
		dataManager->getBoundingBox()->calcBoundingBox();


		std::cout << "ROTATION MATRIX: " << rotM << std::endl; //passt! kann man so nehmen!!!
		std::cout << "CURRENT PROJECTION CENTRE: " << dataManager->getProjectionCenter().x << "," << dataManager->getProjectionCenter().y << "," << dataManager->getProjectionCenter().z << "," << std::endl;
		//dataManager->setProjectionCenter()

	
		// check distribution of matched object points inside image (should have points in each quadric of the image)
		if (dataManager->get_well_distributed_object_points_image_space()) {
			should_I_refine_IOP = true;
			logFilePrinter->append(TAG + "refine IOP: true (have well distributed 2D - 3D point pairs. Refine IO in next iteration)");
		}
		else {
			should_I_refine_IOP = false;
			logFilePrinter->append(TAG + "refine IOP: false (have insuffient distributed 2D - 3D point pairs. No refinement in next iteration)");
		}


		if (iteration == FINAL_ITERATION) {
			repeat_i2g = false;
		}
		else {
			iteration++;
			delete pim;
			delete matching;

			// clear wd
			boost::filesystem::remove_all(dataManager->get_path_working_directory() + "\\myData\\");
			boost::filesystem::remove_all(dataManager->get_path_working_directory() + "\\Matching\\");
			boost::filesystem::remove_all(dataManager->get_path_working_directory() + "\\VegetationMask\\");


			
		}

		//repeat_i2g = false; // TEST


	} //end while (repeate_i2g)






	// 10. projection water line into object space
	logFilePrinter->append(TAG + "Object_points_size: " + std::to_string((*dataManager->get_pts_synth_3D_double()).size()));
	logFilePrinter->append(TAG + "Waterline_image_points_size: " + std::to_string((*dataManager->get_water_line_image_points_2D_ptr()).size()));

	bool print_pcl = false;
	matching->waterlineProjection(*dataManager->get_water_line_image_points_2D_ptr(), *dataManager->get_pts_synth_3D_double(), dataManager->get_true_image(), camera_matrix, dist_coeffs, rvec, tvec, dataManager->get_utm_shift_x(), dataManager->get_utm_shift_y(), print_pcl);


	logFilePrinter->print_content_disk(path_working_directory + "logfile.txt");


	// final call destructors
	delete logFilePrinter;
	delete pim;
	delete dataManager;
	delete matching;

	return (EXIT_SUCCESS);
}

