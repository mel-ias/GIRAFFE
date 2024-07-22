#pragma once

#ifndef MATCHING_H
#define MATCHING_H

#include "ImCalculator.hpp"
#include "Mathematics.h"
#include "LogfilePrinter.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "Modell_OCV.h"
#include "DataManager.h"
#include "Utils.h"

#include <thread>
#endif




class Matching
{

	
public:

	enum OutlierTests {
		NON,HOMO,FUND,HOMO_FUND
	};

	enum Detector {
		MSCR, SIFT, FAST, HARRIS
	};

	enum Flags_resec {
		CALC_EO_IO, CALC_EO, FIXED_EO_IO
	};
	
	// C'tor
	Matching();
	// D'tor
	virtual ~Matching();

	
	// init
	void init(DataManager* _dataManager);
	
	// logic
	void waterlineProjection(std::vector<cv::Point2d>& in_wl_pts_2D, std::vector<Vek3d>& synth_pts_3D, cv::Mat& in_image_4_color, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec_cc_orig_copy, cv::Mat& tvec_cc_orig_copy, double shift_vector_x, double shift_vector_y, double shift_vector_z, bool print_pcl, std::string file_name_image_points);

	void Matching::loadMatches(
		std::string in_path_matching_output,
		cv::Mat& in_real_image,
		cv::Mat& in_synth_image,
		std::vector<cv::Point2d>& in_wl_pts_2D,
		std::vector<Vek2d>& in_synth_pts_2D,
		std::vector<Vek3d>& in_synth_pts_3D,
		std::vector<cv::Point3d>& out_matched_object_points,
		std::vector<cv::Point2d>& out_matched_image_points_real,
		std::vector<cv::Point2d>& out_matched_image_points_synth,
		double d2Net_scalingFactor_trueImage,
		double d2net_scalingFactor_synthImage,
		int flag_matching_type,
		float neighbour_distance_allowed_pointcloud);
	
	
	

	float enhanced_spatial_resection(std::vector<cv::Point3d> &in_matched_object_points, std::vector<cv::Point2d>& in_matched_image_points_real, cv::Mat& real_canvas, double in_pix_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& stdDev_In, cv::Mat& stdDev_Ext, Flags_resec in_flag, bool fisheye);


	
	

	

private:

	// internal functions
	void calculate_nn_synth_key___pts_image_pts(

		const std::vector<Vek2d>& in_synth_pts_float,//punkte
		const std::vector<Vek3d>& in_synth_pts_3D_float, //punkte3d
		const std::vector<Vek2d>& in_synth_keypoints_float,
		const std::vector<Vek2d>& in_real_keypoints_float,

		const cv::Mat& in_real_image,
		const cv::Mat& in_synth_image,

		std::vector<cv::Point2d>& out_matched_image_points_real,
		std::vector<cv::Point2d>& out_matched_image_points_synth,
		std::vector<cv::Point3d>& out_matched_object_points,

		const float in_neighbour_distance
	);

	// Output functions
	void output_enhanced_spatial_resection(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& stdDev_In, cv::Mat& stdDev_Ext, cv::Mat& perViewErrors, double in_pix_size);
	void output_correlation_matrix_intO_extO(cv::Mat &_correlation_matrix, bool aspect_ratio_fx_fy = true);
	
	void draw_matches_reimpl_MK(cv::Mat& canvas_real_image, cv::Mat& canvas_synth_image, std::vector <Vek2d>& real_matches_draw, std::vector <Vek2d>& synth_matches_draw, std::string fileName);
	void draw_valid_matches_after_back_projection(cv::Mat& in_canvas, std::vector<cv::Point2f>& in_image_points_real_ransac, std::vector<cv::Point2f>& in_matched_object_points_ransac_projected);
	void draw_inliers_distribution(cv::Mat& real_image, std::vector<cv::Point2d>& inliers, std::vector<cv::Point2d>& waterline_points, int raster_size, std::string file_name);

	void writer_matches_txt_ellipsoid_bat(std::vector<cv::Point3d>& objectPoints3D,	std::vector<cv::Point2d>& imagePoints2D_realImage,	std::vector<cv::Point2d>& imagePoints2D_synthImage);

	
	void printLogfile_log_statistics(std::stringstream log_statistics) {
		std::string line;
		std::ofstream myfile_stats;
		// Schreibe statisitken raus
		myfile_stats.open(mWorkingDirectory + "waterlinepoints_projected_stats.txt");

		while (std::getline(log_statistics, line)) {
			std::cout << line << endl;
		}
		myfile_stats.close();
	}

	cv::Mat ransacTest_reimpl(std::vector<cv::Point2d>& _points1, std::vector<cv::Point2d>& _points2, double confidence, double distance, bool refineF);


	//Hier Matching - Parameterisierung
	double _confidence = 0.9999998;
	double _distance = 1.0;

		// constants
	std::string TAG = "Matching:\t";
	

	// member
	DataManager *mDataManager;
	LogFile *mLogFile;
	std::string mWorkingDirectory;
	float mReproError4JSON = 0.0f; // copy of repro_error after camera calibration for json output


	// global definition for concern in output
	double inliers_per_importance_cell_percent = 0.0;
	double point_counter_importance = 0.0;
	int no_inliers_4_stat = 0;

	
};


