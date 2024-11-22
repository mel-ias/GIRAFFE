#pragma once

#ifndef MATCHING_H
#define MATCHING_H

#include "ImCalculator.hpp"
#include "LogfilePrinter.h"
#include "Model.h"
#include "DataManager.h"
#include "Utils.h"

#include <algorithm>
#include <thread>
#include <vector>
#include <cassert>
#include <string>
#include <utility>

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
	
	/**
	 * @brief Default constructor for the Matching class.
	 *
	 * Initializes the Matching class members to default values. Sets the data manager and log file pointers to nullptr
	 * and initializes the working directory path as an empty string.
	 */
	Matching();
	

	/**
	 * @brief Destructor for the Matching class.
	 *
	 * Ensures any allocated resources are properly cleaned up when a Matching object is destroyed.
	 */
	virtual ~Matching();

	
	/**
	 * @brief Initializes the Matching class by setting up the log file, data manager, and working directory.
	 *
	 * This function sets up the internal log file from the provided DataManager, logs the initialization process,
	 * and establishes a working directory for the matching operation. If the directory does not already exist,
	 * it is created within the specified working directory path.
	 *
	 * @param[in] _dataManager Pointer to the DataManager object that provides access to logging and file paths.
	 */
	void init(DataManager* _dataManager);
	

	/**
	 * @brief Projects 2D image points to 3D space, colors a synthetic point cloud, and optionally exports the point cloud and projections.
	 *
	 * This function undistorts 2D input image points, references them against a synthetic 3D point cloud,
	 * colors the points based on an input image, and optionally exports the recolored point cloud and
	 * projected 2D points. The function logs the process and results.
	 *
	 * @param[in] input_image_points         Vector of 2D input image points.
	 * @param[in] synth_pts_3D               Vector of synthetic 3D points in the point cloud.
	 * @param[in] in_image_4_color           Input image used for coloring the points.
	 * @param[in] camera_matrix              Camera matrix for projection.
	 * @param[in] dist_coeffs                Distortion coefficients for undistorting points.
	 * @param[in] rvec_cc_orig_copy          Rotation vector for coordinate transformation.
	 * @param[in] tvec_cc_orig_copy          Translation vector for coordinate transformation.
	 * @param[in] shift_x                    X-axis shift for exported points.
	 * @param[in] shift_y                    Y-axis shift for exported points.
	 * @param[in] shift_z                    Z-axis shift for exported points.
	 * @param[in] export_pcl                 Flag indicating whether to export the point cloud.
	 * @param[in] file_name_image_points     Filename prefix for exporting projected points.
	 */
	void image_points_3D_referencing(
		std::vector<std::vector<cv::Point2d>>& input_image_points,
		std::vector<cv::Point3d>& synth_pts_3D,
		cv::Mat& in_image_4_color, 
		cv::Mat& camera_matrix, 
		cv::Mat& dist_coeffs,
		cv::Mat& rvec_cc_orig_copy, 
		cv::Mat& tvec_cc_orig_copy, 
		double shift_vector_x,
		double shift_vector_y,
		double shift_vector_z, 
		bool print_pcl, 
		std::vector<std::string> file_name_image_points);


	/**
	 * @fn	void Matching::loadMatches( std::string in_path_FMatrixOutput, cv::Mat&amp; in_real_image, cv::Mat&amp; in_synth_image, std::vector&lt;cv::Point2d&gt;&amp; in_wl_pts_2D, std::vector&lt;cv::Point2d&gt;&amp; in_synth_pts_2D, std::vector&lt;cv::Point3d&gt;&amp; in_synth_pts_3D, std::vector&lt;cv::Point3d&gt;&amp; out_matched_object_points, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_real, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_synth)
	 *
	 * @summary	public function read f-inlier matches from SiftGPU (included in Visual SFM console
	 * 			application)
	 *
	 * @param 			in_path_FMatrixOutput	  	input string path to matching results from Visual SFM.
	 * @param [in]		in_real_image				  	matrix of real image.
	 * @param [in]		in_synth_image				  	matrix of rendered image.
	 * @param [in]		in_synth_pts_2D				  	image points of the rendered image
	 * @param [in]		in_synth_pts_3D				  	object points that corresponds to the image	points of the rendered image
	 * @param [out]	out_matched_object_points	  	image points from the real image having corresponding keypoints inside the rendered image with valid 3D correspondence
	 * @param [out]	out_matched_image_points_real 	image points from the rendered image having corresponding keypoints inside the real image with valid 3D correspondence
	 * @param [out]	out_matched_image_points_synth	object points that have corresponding points within the real and the rendered image.
	 */
	void loadMatches(
		std::string in_path_matching_output,
		cv::Mat& in_real_image,
		cv::Mat& in_synth_image,
		std::vector<cv::Point2d>& in_synth_pts_2D,
		std::vector<cv::Point3d>& in_synth_pts_3D,
		std::vector<cv::Point3d>& out_matched_object_points,
		std::vector<cv::Point2d>& out_matched_image_points_real,
		std::vector<cv::Point2d>& out_matched_image_points_synth,
		float neighbour_distance_allowed_pointcloud = 2.5); // neighbour distance in pixels
	

	/**
	 * @brief Prüft, ob die Verteilung der Punkte über das Bild ausreichend ist.
	 *
	 * Diese Funktion teilt das Bild in ein Raster und überprüft, ob jede Zelle
	 * eine Mindestanzahl an Punkten enthält, um eine gute Verteilung zu gewährleisten.
	 *
	 * @param imagePoints 2D-Punkte im Bild.
	 * @param imageSize Größe des Bildes.
	 * @param gridRows Anzahl der Zeilen im Raster.
	 * @param gridCols Anzahl der Spalten im Raster.
	 * @param minPointsPerCell Mindestanzahl an Punkten pro Zelle.
	 * @return True, wenn die Verteilung gut ist, false andernfalls.
	 */
	bool is_distribution_good(
		const std::vector<cv::Point2f>& imagePoints,
		const cv::Size& imageSize,
		int gridRows = 4,
		int gridCols = 4,
		int minPointsPerCell = 3);


	/**
	 * @brief Calibrates a camera using 3D-2D point correspondences and computes intrinsic and extrinsic parameters.
	 *
	 * This function performs camera calibration given a set of 3D object points and their corresponding 2D image points.
	 * It can calibrate either a standard pinhole camera model or a fisheye lens model, based on the `useFisheye` parameter.
	 * The function outputs the camera's intrinsic matrix (`cameraMatrix`), distortion coefficients (`distCoeffs`),
	 * and optionally rotation (`rvecs`) and translation (`tvecs`) vectors for each image.
	 *
	 * @param objectPoints 3D points in the world coordinate space for each image.
	 * @param imagePoints 2D points in the image plane corresponding to `objectPoints`.
	 * @param imageSize Size of the images used for calibration.
	 * @param cameraMatrix Output matrix containing intrinsic camera parameters.
	 * @param distCoeffs Output vector of distortion coefficients.
	 * @param useFisheye If true, enables fisheye lens calibration; otherwise, standard pinhole model calibration is used.
	 * @return True if the calibration succeeded, false otherwise.
	 */
	bool run_calibration(
		std::vector<std::vector<cv::Point3f>> objectPoints,
		std::vector<std::vector<cv::Point2f>> imagePoints,
		cv::Size& imageSize,
		cv::Mat& cameraMatrix,
		cv::Mat& distCoeffs,
		bool useFisheye);


	/**
	 * @brief Space resection using matched object and image points to estimate intrinsic / extrinsic camera parameters.
	 *
	 * This function refines the extrinsic and intrinsic parameters of the camera using spatial resection techniques.
	 * It supports both fisheye and pinhole camera models and performs outlier rejection using solvePnPRansac.
	 * Optionally, it refines the solution using solvePnPRefineLM or calibrateCamera based on the input flags.
	 *
	 * @param[in] in_matched_object_points Matched 3D object points (in world coordinates).
	 * @param[in] in_matched_image_points_real Matched 2D image points (in image coordinates).
	 * @param[in] true_image Original image matrix for size reference.
	 * @param[in] camera_matrix Intrinsic camera matrix.
	 * @param[in] dist_coeffs Camera distortion coefficients.
	 * @param[out] rvec Rotation vector (output if solved).
	 * @param[out] tvec Translation vector (output if solved).
	 * @param[out] stdDev_In Standard deviation of intrinsic parameters.
	 * @param[out] stdDev_Ext Standard deviation of extrinsic parameters.
	 * @param[in] in_flag Flag indicating which parameters to optimize (extrinsics, intrinsics, or both).
	 * @param[in] fisheye Boolean flag indicating whether a fisheye model is used.
	 *
	 * @return The reprojection error after calibration or -1 if an error occurs.
	 */
	double space_resection(
		std::vector<cv::Point3d> &in_matched_object_points, 
		std::vector<cv::Point2d>& in_matched_image_points_real, 
		cv::Mat& real_canvas, 
		cv::Mat& camera_matrix, 
		cv::Mat& dist_coeffs, 
		cv::Mat& rvec, 
		cv::Mat& tvec, 
		cv::Mat& stdDev_In, 
		cv::Mat& stdDev_Ext, 
		Flags_resec in_flag, 
		bool fisheye);

	

private:

	struct FilteredData {
	public:
		std::vector<int> image_data_original_idx;
		std::vector<cv::Point2d> image_data_original;
		std::vector<cv::Point2d> image_data_projected;
		std::vector<cv::Point3d> image_data_3D;
		
	};


	/**
	 * @brief Calculates the nearest neighbor matches between synthesized keypoints and real keypoints using KNN search.
	 *
	 * This function finds the closest synthesized points corresponding to the synthesized keypoints
	 * from an image and matches them with real keypoints. It also returns the matched 3D points.
	 * Points are matched based on a specified distance threshold.
	 *
	 * @param in_synth_pts_float A vector of synthesized 2D points (in image space).
	 * @param in_synth_pts_3D_float A vector of synthesized 3D points (object space).
	 * @param in_synth_keypoints_float A vector of synthesized keypoints (2D).
	 * @param in_real_keypoints_float A vector of real keypoints (2D).
	 * @param in_real_image The real image for visualization.
	 * @param in_synth_image The synthesized image for visualization.
	 * @param out_matched_image_points_real Output vector for matched real image points.
	 * @param out_matched_image_points_synth Output vector for matched synthesized image points.
	 * @param out_matched_object_points Output vector for matched 3D object points.
	 * @param in_neighbour_distance The distance threshold for matching (in pixels).
	 */
	void calculate_nn_synth_key___pts_image_pts(
		const std::vector<cv::Point2d>& in_synth_pts_float,//punkte
		const std::vector<cv::Point3d>& in_synth_pts_3D_float, //punkte3d
		const std::vector<cv::Point2d>& in_synth_keypoints_float,
		const std::vector<cv::Point2d>& in_real_keypoints_float,
		const cv::Mat& in_real_image,
		const cv::Mat& in_synth_image,
		std::vector<cv::Point2d>& out_matched_image_points_real,
		std::vector<cv::Point2d>& out_matched_image_points_synth,
		std::vector<cv::Point3d>& out_matched_object_points,
		const float in_neighbour_distance = 2.5 // distance in pixels
	);


	/**
	 * @brief Applies RANSAC to estimate the fundamental matrix and refines it using homography if needed.
	 *
	 * This function takes two sets of matching points and uses RANSAC to estimate the fundamental matrix.
	 * It filters outliers, refines the results by recomputing the fundamental matrix with inliers, and
	 * optionally applies homography for further refinement.
	 *
	 * @param _points1 Vector of 2D points from the first image.
	 * @param _points2 Vector of 2D points from the second image.
	 * @param confidence Confidence level for the RANSAC algorithm (typical values range between 0.95 to 0.99).
	 * @param distance Distance threshold to decide if a point is an inlier based on the epipolar constraint.
	 * @param refineF Boolean flag to indicate whether the fundamental matrix should be recomputed after inliers are found.
	 *
	 * @return cv::Mat The fundamental matrix estimated by RANSAC using the input points.
	 *
	 * @note The function assumes that the points in both input vectors are already matched.
	 * It filters the inliers from the initial matches and optionally refines the result.
	 */
	cv::Mat ransac_test(
		std::vector<cv::Point2d>& _points1, 
		std::vector<cv::Point2d>& _points2, 
		double confidence = 0.95, 
		double distance = 8.0,
		bool refineF = false);


	// Function to calculate the Euclidean distance between two cv::Point2d points
	double euclidean_distance(const cv::Point2d& p1, const cv::Point2d& p2) {
		return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
	}


	/**
	 * @brief Filters pairs of 2D image points and their corresponding 3D points based on Euclidean distance.
	 *
	 * This function iterates over two lists of 2D image points and filters them by comparing each pair's
	 * distance to a specified threshold. Points within the threshold distance are retained along with
	 * their 3D coordinates and indices. This is useful for eliminating outliers in 3D referencing.
	 *
	 * @param[in] list1             First list of 2D image points (e.g., original points).
	 * @param[in] list2             Second list of 2D image points (e.g., projected points).
	 * @param[in] list2_3d          Corresponding list of 3D points for the second list.
	 * @param[in] distanceThreshold Maximum allowable Euclidean distance for a pair to be retained.
	 *
	 * @return FilteredData         A structure containing filtered indices, 2D points from list1 and list2,
	 *                              and their corresponding 3D points from list2_3d.
	 */
	FilteredData filter_pts_by_distance(
		const std::vector<cv::Point2d>& list1,
		const std::vector<cv::Point2d>& list2,
		const std::vector<cv::Point3d>& list2_3d,
		double distanceThreshold = 2.0);


	// Output functions
	/**
	 * @brief Outputs enhanced camera calibration results to the log file.
	 *
	 * This method logs the detailed camera parameters, including both intrinsic and extrinsic calibration
	 * data, along with their associated standard deviations. The output is formatted and includes both pixel
	 * and millimeter measurements based on the provided pixel size.
	 *
	 * @param in_camera_matrix  The camera intrinsic matrix (3x3).
	 * @param in_dist_coeffs    The camera distortion coefficients.
	 * @param in_rvec           The rotation vector obtained from the calibration process.
	 * @param in_tvec           The translation vector obtained from the calibration process.
	 * @param in_StdDev_IntO    Standard deviations of intrinsic parameters.
	 * @param in_StdDev_ExtO    Standard deviations of extrinsic parameters.
	 * @param in_PerViewErrors  Per-view reprojection errors.
	 * @param in_pix_size       The size of a pixel in millimeters, used to convert pixel measurements.
	 */
	void write_camera_calibration_statistics(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& stdDev_In, cv::Mat& stdDev_Ext, cv::Mat& perViewErrors, double in_pix_size);
	

	/**
	 * @brief Draws matches between real and synthetic images, and saves the result as an image file.
	 *
	 * This function visualizes the matches between keypoints from a real image and a synthetic image.
	 * It draws lines connecting the corresponding points from both images and saves the result in the specified file.
	 *
	 * @param[in] in_real_image The real image (as an OpenCV matrix).
	 * @param[in] in_synth_image The synthetic image (as an OpenCV matrix).
	 * @param[in] in_real_matches_draw A vector of 2D keypoints from the real image.
	 * @param[in] in_synth_matches_draw A vector of 2D keypoints from the synthetic image.
	 * @param[in] fileName The name of the output file (without extension) where the result will be saved.
	 */
	void write_visualization_matches(cv::Mat& canvas_real_image, cv::Mat& canvas_synth_image, std::vector <cv::Point2d>& real_matches_draw, std::vector <cv::Point2d>& synth_matches_draw, std::string fileName);
	

	/**
	 * @brief Writes corresponding 3D object points and 2D image points (real and synthetic) to text files.
	 *
	 * This function writes the provided 3D object points and their corresponding 2D real and synthetic image points
	 * to separate text files. It ensures that the number of points in all three vectors matches before writing.
	 * The output files contain the points with a specific format, where each entry is prefixed with an identifier.
	 *
	 * @param[in,out] object_points_3D A vector of 3D points representing object coordinates.
	 * @param[in,out] image_points_2D_real A vector of 2D points representing real image coordinates.
	 * @param[in,out] image_points_2D_synth A vector of 2D points representing synthetic image coordinates.
	 *
	 * @note The function assumes that the three input vectors have the same size. If the sizes do not match,
	 * the function will log an error and return without writing the files.
	 */
	void write_corresponding_points_to_file(const std::vector<cv::Point3d>& object_points_3D, const std::vector<cv::Point2d>& image_points_2D_real, const std::vector<cv::Point2d>& image_points_2D_synth);
	

	/**
	 * @brief Writes log statistics to a file and prints them to the console.
	 *
	 * This method extracts log data from a stringstream and writes it to a log file, while also printing each line
	 * to the console for immediate feedback. The log file is saved in the working directory under the name
	 * "image_points_projected.txt".
	 *
	 * @param log_statistics A stringstream containing the statistics to be logged.
	 */
	void write_and_print_log_statistics(std::stringstream& log_statistics) const;


	// constants
	std::string TAG = "Matching:\t";

	// member
	DataManager *_data_manager;
	LogFile *_logfile;
	fs::path _working_dir_matching;
		
};


