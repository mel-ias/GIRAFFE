#pragma once

#ifndef MATCHING_H
#define MATCHING_H

#include "ImCalculator.hpp"
#include "LogfilePrinter.h"
#include "Modell_OCV.h"
#include "DataManager.h"
#include "Utils.h"
#include "vek.h"

#include <algorithm>
#include <thread>
#include <vector>
#include <cassert>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/flann.hpp>

#endif




class Matching
{

	
public:

	
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
	void image_points_3D_referencing(
		std::vector<cv::Point2d>& in_wl_pts_2D, 
		std::vector<Vek3d>& synth_pts_3D, 
		cv::Mat& in_image_4_color, 
		cv::Mat& camera_matrix, 
		cv::Mat& dist_coeffs,
		cv::Mat& rvec_cc_orig_copy, 
		cv::Mat& tvec_cc_orig_copy, 
		double shift_vector_x,
		double shift_vector_y,
		double shift_vector_z, 
		bool print_pcl, 
		std::string file_name_image_points, 
		double tolerance_distortion_px = 5.0);

	void loadMatches(
		std::string in_path_matching_output,
		cv::Mat& in_real_image,
		cv::Mat& in_synth_image,
		std::vector<cv::Point2d>& in_wl_pts_2D,
		std::vector<Vek2d>& in_synth_pts_2D,
		std::vector<Vek3d>& in_synth_pts_3D,
		std::vector<cv::Point3d>& out_matched_object_points,
		std::vector<cv::Point2d>& out_matched_image_points_real,
		std::vector<cv::Point2d>& out_matched_image_points_synth,
		float neighbour_distance_allowed_pointcloud = 2.5); // neighbour distance in pixels
	

	float space_resection(
		std::vector<cv::Point3d> &in_matched_object_points, 
		std::vector<cv::Point2d>& in_matched_image_points_real, 
		cv::Mat& real_canvas, 
		double in_pix_size, 
		cv::Mat& camera_matrix, 
		cv::Mat& dist_coeffs, 
		cv::Mat& rvec, 
		cv::Mat& tvec, 
		cv::Mat& stdDev_In, 
		cv::Mat& stdDev_Ext, 
		Flags_resec in_flag, 
		bool fisheye);

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
		const float in_neighbour_distance = 2.5 // distance in pixels
	);

	cv::Mat ransac_test(
		std::vector<cv::Point2d>& _points1, 
		std::vector<cv::Point2d>& _points2, 
		double confidence = 0.95, 
		double distance = 8.0,
		bool refineF = false);


	// Output functions
	void write_camera_calibration_statistics(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& stdDev_In, cv::Mat& stdDev_Ext, cv::Mat& perViewErrors, double in_pix_size);
	void write_visualization_matches(cv::Mat& canvas_real_image, cv::Mat& canvas_synth_image, std::vector <Vek2d>& real_matches_draw, std::vector <Vek2d>& synth_matches_draw, std::string fileName);
	void write_corresponding_points_to_file(std::vector<cv::Point3d>& objectPoints3D, std::vector<cv::Point2d>& imagePoints2D_realImage,	std::vector<cv::Point2d>& imagePoints2D_synthImage);
	void write_and_print_log_statistics(std::stringstream& log_statistics);

	// constants
	std::string TAG = "Matching:\t";

	// member
	DataManager *mDataManager;
	LogFile *mLogFile;
	std::string mWorkingDirectory;
		
};


