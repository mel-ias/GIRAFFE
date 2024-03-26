/**
 * @file	Matching.cpp.
 *
 * @brief	Implements the matching class.
 */

#include "Matching.h"

 /**
  * @fn	Matching::Matching()
  *
  * @summary	Matching class. Contains functions for image-to-geometry intersection.
  *
  * @author	M. Kroehnert
  * @date	25.04.2018
  */

Matching::Matching() {

}

/**
 * @fn	Matching::~Matching()
 *
 * @brief	Destructor.
 *
 * @author	M. Kroehnert
 * @date	25.04.2018
 */

Matching::~Matching() {

}

// hand over data manager and calculated synthetic image 

/**
 * @fn	void Matching::init(DataManager* _dataManager)
 *
 * @summary	initialisation Matching instance, create working directory \\Matching
 *
 * @author	M. Kroehnert
 * @date	25.04.2018
 *
 * @param [in]	_dataManager	Input data base, instance of DataManager storing all attributes from input data and results from calculations.
 */

void Matching::init(DataManager* _dataManager) {

	// initialisation log_file
	mLogFile = _dataManager->getLogFilePrinter();
	mLogFile->append("");
	mLogFile->append(TAG + "---- initialisation matching ----");

	// hand over data manager
	mDataManager = _dataManager;

	// create working directory .../Matching
	mWorkingDirectory = (mDataManager->get_path_working_directory() + "\\Matching\\");
	CreateDirectoryA(LPCSTR(mWorkingDirectory.c_str()), NULL);

}

/**
 * @fn	void Matching::calculate_nn_synth_key___pts_image_pts( const std::vector&lt;Vek2d&gt;&amp; in_synth_pts_float, const std::vector&lt;Vek3d&gt;&amp; in_synth_pts_3D_float, const std::vector&lt;Vek2d&gt;&amp; in_synth_keypoints_float, const std::vector&lt;Vek2d&gt;&amp; in_real_keypoints_float, const cv::Mat&amp; in_real_image, const cv::Mat&amp; in_synth_image, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_real, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_synth, std::vector&lt;cv::Point3d&gt;&amp; out_matched_object_points, const float in_neighbour_distance = 0.75)
 *
 * @summary	private function call function after matching of a real and a synthetic image
 * 			scene. check if image matches have a valid correspondance in 3d object space using knn
 * 			search with threshold for maximum distance.
 *
 * @author	M. Kroehnert
 *
 * @date	25.04.2018
 *
 * @param [in]		in_synth_pts_float			  	image points from the rendered image.
 * @param [in]		in_synth_pts_3D_float		  	object points corresponding to image points from the rendered image
 * @param [in]		in_synth_keypoints_float	  	image points from the rendered image having corresponding keypoints inside the real image
 * @param [in]		in_real_keypoints_float		  	image points from the real image having corresponding keypoints inside the rendered	image
 * @param [in]		in_real_image				  	image matrix of real image
 * @param [in]		in_synth_image				  	image matrix of rendered image
 * @param [out]	out_matched_image_points_real 	points from the real image having corresponding keypoints inside the rendered image with valid 3D correspondence
 * @param [out]	out_matched_image_points_synth	image points from the rendered image having	corresponding keypoints inside the real image with valid 3D correspondence.
 * @param [out]	out_matched_object_points	  	object points that have corresponding points within the real and the rendered image.
 * @param [in]		in_neighbour_distance		  	(Optional) define distance threshold for corresponding points using knn search in object space (standard = 0.75)
 */

void Matching::calculate_nn_synth_key___pts_image_pts(

	const std::vector<Vek2d>& in_synth_pts_float,//punkte
	const std::vector<Vek3d>& in_synth_pts_3D_float, //punkte3d
	const std::vector<Vek2d>& in_synth_keypoints_float,
	const std::vector<Vek2d>& in_real_keypoints_float,

	const cv::Mat& in_real_image,
	const cv::Mat& in_synth_image,

	std::vector<cv::Point2d>& out_matched_image_points_real,
	std::vector<cv::Point2d>& out_matched_image_points_synth,
	std::vector<cv::Point3d>& out_matched_object_points,

	const float in_neighbour_distance = 0.75) {

	// check input data
	assert(!in_synth_pts_float.empty());
	assert(!in_synth_pts_3D_float.empty());

	// clear vectors (for the case that there are already existing values for matched_image_points_synth/real and matched_object_points)
	out_matched_image_points_synth.clear();
	out_matched_image_points_real.clear();
	out_matched_object_points.clear();

	// append log_file
	mLogFile->append(TAG + "count synth_keypoints_float: " + std::to_string(in_synth_keypoints_float.size()));
	mLogFile->append(TAG + "count real_keypoints_float: " + std::to_string(in_real_keypoints_float.size()));
	mLogFile->append(TAG + "neighbour distance threshold (in Px, synth match to surrounding waiting for synth point with 3D data serving as reference point): " + std::to_string(in_neighbour_distance));


	// run kd_tree to determine nearest neighbour of matched synth_keypoint by synth_image points
	KdTree2d kdtree(in_synth_pts_float, 12, 500); // 12 = node depth, 500 = iteration, i.e. after 500 matches --> break
	kdtree.init();

	if (!kdtree.istInitialisiert()) {
		mLogFile->append(TAG + "cannot initialize KDtree. return");
		return;
	}

	// init canvas for output
	cv::Mat draw_canvas_synth;
	in_synth_image.copyTo(draw_canvas_synth);

	cv::Mat draw_canvas_real;
	in_real_image.copyTo(draw_canvas_real);


	// step through std::vector<Vek2d>& _synth_keypoints_float and apply kd tree
	// get for each synth keypoint (Vek2d p) corresponding synth image pixel that fulfills condition _neighbour_distance
	uint counter = 0;
	for (auto& p : in_synth_keypoints_float) {

		// find nn for integer (whole number) key point browse _synth_pts_float
		auto nachbar = kdtree.nn(p);

		// ensure that neighbour distance is valid (acceptable distance standard = 1.5 px), otherwise continue
		if (nachbar.distanz > in_neighbour_distance) {
			counter++; // increment counter in each case
			continue;
		}

		// push back corresponding synth_2D, real_2D (image points) and synth_3D (object points) 
		out_matched_image_points_synth.push_back(cv::Point2d(in_synth_pts_float[nachbar.idx].x(), in_synth_pts_float[nachbar.idx].y()));
		out_matched_image_points_real.push_back(cv::Point2d(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()));
		out_matched_object_points.push_back(cv::Point3d(in_synth_pts_3D_float[nachbar.idx].x(), in_synth_pts_3D_float[nachbar.idx].y(), in_synth_pts_3D_float[nachbar.idx].z()));

		// output
		cv::circle(draw_canvas_synth, cv::Point(in_synth_pts_float[nachbar.idx].x(), in_synth_pts_float[nachbar.idx].y()), 5, cv::Scalar(0, 0, 255), -1);
		cv::circle(draw_canvas_synth, cv::Point(p.x(), p.y()), 1, cv::Scalar(0, 255, 0), 3);
		cv::circle(draw_canvas_real, cv::Point(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()), 1, cv::Scalar(0, 255, 0), 3);

		std::string counterTxt = std::to_string(counter) + "," + std::to_string(in_synth_pts_3D_float[nachbar.idx].x()) + "," + std::to_string(in_synth_pts_3D_float[nachbar.idx].y()) + "," + std::to_string(in_synth_pts_3D_float[nachbar.idx].z());
		cv::putText(draw_canvas_real, std::to_string(counter), cv::Point(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
		cv::putText(draw_canvas_synth, std::to_string(counter), cv::Point(in_synth_pts_float[nachbar.idx].x(), in_synth_pts_float[nachbar.idx].y()), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
		// -output

		counter++;  // increment counter in each case
	}

	// append log_file/ output
	mLogFile->append(TAG + "KDtree: size matched_objectPoints/ matched_imagePoints_synth/ matched_imagePoints_real: " + std::to_string(out_matched_object_points.size()) + "/" + std::to_string(out_matched_image_points_synth.size()) + "/" + std::to_string(out_matched_image_points_real.size()));
	mLogFile->append(TAG + "finish matching");
	cv::imwrite(mWorkingDirectory + "corresp_points_synth.png", draw_canvas_synth);
	cv::imwrite(mWorkingDirectory + "corresp_points_real.png", draw_canvas_real);


	//std::cout << "Try DLT!" << std::endl;
	//calcDLT(imagePoints_real, objectPoints);

}






/**
 * @fn	void Matching::loadMatches( std::string in_path_FMatrixOutput, cv::Mat&amp; in_real_image, cv::Mat&amp; in_synth_image, std::vector&lt;cv::Point2d&gt;&amp; in_wl_pts_2D, std::vector&lt;Vek2d&gt;&amp; in_synth_pts_2D, std::vector&lt;Vek3d&gt;&amp; in_synth_pts_3D, std::vector&lt;cv::Point3d&gt;&amp; out_matched_object_points, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_real, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_synth)
 *
 * @summary	public function read f-inlier matches from SiftGPU (included in Visual SFM console
 * 			application)
 *
 * @author	M. Kroehnert
 *
 * @date	25.04.2018
 *
 * @param 			in_path_FMatrixOutput	  	input string path to matching results from Visual SFM.
 * @param [in]		in_real_image				  	matrix of real image.
 * @param [in]		in_synth_image				  	matrix of rendered image.
 * @param [in]		in_wl_pts_2D					image points of the detected water line
 * @param [in]		in_synth_pts_2D				  	image points of the rendered image
 * @param [in]		in_synth_pts_3D				  	object points that corresponds to the image	points of the rendered image
 * @param [out]	out_matched_object_points	  	image points from the real image having corresponding keypoints inside the rendered image with valid 3D correspondence
 * @param [out]	out_matched_image_points_real 	image points from the rendered image having corresponding keypoints inside the real image with valid 3D correspondence
 * @param [out]	out_matched_image_points_synth	object points that have corresponding points within the real and the rendered image.
 */
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
	double scalingFactor_trueImage,
	double scalingFactor_synthImage,
	int flag_matching_type,
	float neighbour_distance_allowed_pointcloud) {

	mLogFile->append("");
	mLogFile->append(TAG + "read matching results");
	mLogFile->append(TAG + "real_image: " + std::to_string(in_real_image.size().width) + "x" + std::to_string(in_real_image.size().height) + "," + std::to_string(in_real_image.type()));
	mLogFile->append(TAG + "synth_image: " + std::to_string(in_synth_image.size().width) + "x" + std::to_string(in_synth_image.size().height) + "," + std::to_string(in_synth_image.type()));

	// initialisation
	uint counter = 0;
	std::ifstream file(in_path_matching_output);
	std::string line;
	std::vector<cv::Point2d> real_matched_pts, synth_matched_pts;


	// read line-by-line
	while (std::getline(file, line)) {

		if (flag_matching_type == DataManager::D2NET || flag_matching_type == DataManager::SUPERGLUE || flag_matching_type == DataManager::LIGHTGLUE) {

			// create an input string stream
			std::istringstream stm(line);
			int id;
			double img1_x, img1_y, img2_x, img2_y; //coordinates of feature point inliers; img1 = real image; img2 = synth image
			stm >> id >> img1_x >> img1_y >> img2_x >> img2_y;


			// rescale coordinates from matched images
			// trueImage 
			img1_x = img1_x * 1.0 / scalingFactor_trueImage;
			img1_y = img1_y * 1.0 / scalingFactor_trueImage;
			// synthImage 
			img2_x = img2_x * 1.0 / scalingFactor_synthImage;
			img2_y = img2_y * 1.0 / scalingFactor_synthImage;

			// push back inlieres separately
			real_matched_pts.push_back(cv::Point2d(img1_x, img1_y));
			synth_matched_pts.push_back(cv::Point2d(img2_x, img2_y));

			//std::cout << "Imgpoints_DEBUG: " << img1_x << "," << img1_y << "," << img2_x << "," << img2_y << std::endl;

			counter++;
		}
	}

	mLogFile->append(TAG + "count fundamental inlier matches (real_matched_pts/synth_matched_pts):" + std::to_string(real_matched_pts.size()) + "/" + std::to_string(synth_matched_pts.size()));

	// check for outlieres
	double confidence = 0.95;
	double distance = 8.0;
	AccurateMatcher::ransacTest_reimpl(real_matched_pts, synth_matched_pts, confidence, distance, false);


	// data conversion for image matching
	std::vector<Vek2d> real_matched_pts_Vek2d, synth_matched_pts_Vek2d;
	for (cv::Point p : real_matched_pts)
		real_matched_pts_Vek2d.push_back(Vek2d(p.x, p.y));

	for (cv::Point p : synth_matched_pts)
		synth_matched_pts_Vek2d.push_back(Vek2d(p.x, p.y));

	// --- receive 3D coordinates for all image points 
	// vectors for matching results in declaration
	calculate_nn_synth_key___pts_image_pts(
		in_synth_pts_2D, in_synth_pts_3D, synth_matched_pts_Vek2d, real_matched_pts_Vek2d,
		in_real_image, in_synth_image,
		out_matched_image_points_real, out_matched_image_points_synth, out_matched_object_points,
		neighbour_distance_allowed_pointcloud);


	// check sizes of matched point vectors (2D synth, real, 3D synth_object), must be equal otherwise return
	if (out_matched_object_points.size() == 0 || out_matched_image_points_real.size() == 0 || out_matched_image_points_real.size() != out_matched_object_points.size()) {
		mLogFile->append("Problem with determination of corresponding points between matched synthetic and matched real image and 3D point cloud data.");
		return;
	}

	// visualisation of inlier matches distribution for rectification of real image (related to water line points, so only run function if water line points are given!)
	if (!in_wl_pts_2D.empty()) {
		int raster_size = 2;
		draw_inliers_distribution(in_real_image, out_matched_image_points_real, in_wl_pts_2D, raster_size, "matching_distribution");
	}

	// output to ascii point cloud file (matched_object_points, matched_image_points_real, matched_imagePoints_synth)
	writer_matches_txt_ellipsoid_bat(out_matched_object_points, out_matched_image_points_real, out_matched_image_points_synth);

	// draw matches
	draw_matches_reimpl_MK(in_real_image, in_synth_image, real_matched_pts_Vek2d, synth_matched_pts_Vek2d, "matches_2D");

	// update 02.07.22 because function above do not print inlier matches alone!
	std::vector<Vek2d> out_matched_image_points_real_Vek2d, out_matched_image_points_synth_Vek2d;
	for (cv::Point p : out_matched_image_points_real)
		out_matched_image_points_real_Vek2d.push_back(Vek2d(p.x, p.y));

	for (cv::Point p : out_matched_image_points_synth)
		out_matched_image_points_synth_Vek2d.push_back(Vek2d(p.x, p.y));
	draw_matches_reimpl_MK(in_real_image, in_synth_image, out_matched_image_points_real_Vek2d, out_matched_image_points_synth_Vek2d, "matches_with_3D_val");

}





/**
 * @fn	void Matching::enhanced_spatial_resection(std::vector&lt;cv::Point3d&gt; &amp;in_matched_object_points, std::vector&lt;cv::Point2d&gt;&amp; in_matched_image_points_real, cv::Mat&amp; real_canvas, double in_pix_size, cv::Mat&amp; camera_matrix, cv::Mat&amp; dist_coeffs, cv::Mat&amp; rvec, cv::Mat&amp; tvec, Flags_resec in_flag)
 *
 * @summary	Enhanced spatial resection. Determines exterior and interior orientation from given point correspondences 3D geometry <-> 2D image
 * @author		M. Kroehnert
 * @date		25.04.2018
 *
 * @param [in]	in_matched_object_points		[input] matched object points.
 * @param [in]	in_matched_image_points_real	[input] matched image points of (real) camera image
 * @param [in,out]	real_canvas					[input,output] real camera image
 * @param [in]	in_pix_size						[input] pixel size/pitch
 * @param [in,out]	camera_matrix				[input,output] camera matrix corresponding to (real) smartphone image
 * @param [in,out]	dist_coeffs					[input,output] distortion coefficients corresponding to (real) smartphone image
 * @param [in,out]	rvec						[input,output] rotation vector/matrix
 * @param [in,out]	tvec						[input,output] translation vector
 * @param [in] in_flag							[input] flag for input data characterisation:
 * 											FIX_NOTHING = intrincics and extrinsics will be determined,
 * 											FIX_INTR = intrinsics are fixed,  extrinsics will be determined,
 * 											FIX_EXTR = extrinsics are fixed, intrinsics will be determined,
 * 											FIX_INTR_EXTR = all parameters are fixed, no need of re-calculation
 */

float Matching::enhanced_spatial_resection(std::vector<cv::Point3d> &in_matched_object_points, std::vector<cv::Point2d>& in_matched_image_points_real, cv::Mat& true_image, double in_pix_size, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec, cv::Mat& tvec, cv::Mat& stdDev_In, cv::Mat& stdDev_Ext, Flags_resec in_flag, bool fisheye) {



	// multi-function camera re-calculation
	// 1. check if parameters are valid!
	if (camera_matrix.total() != 9) {
		mLogFile->append(TAG + ", camera matrix not valid. please check number of elements.");
		return -1;
	}

	if (dist_coeffs.total() == 0) {
		mLogFile->append(TAG + ", no distortion coefficients provided. please check number of elements.");
		return -1;
	}

	if (in_matched_object_points.size() < 4 || in_matched_image_points_real.size() < 4) {
		mLogFile->append(TAG + ", number of matched object and image points < 4 not allowed.");
		return -1;
	}


	// in case of no extrinsics, need to calculate from object-image point correspondences

	// remove [image point - object point] outliers using fundamental mat calculation due to solvePnPRansac
	// set ransac threshold to 8 px because of initial camera matrix, dist_coeffs and extrinsics (having errors)
	std::vector<int> inliers;
	float reproFisheye = mDataManager->get_filter_matches_ransac_fisheye();
	float reproPinhole = mDataManager->get_filter_matches_ransac_pinhole();
	float solvePnPRansac_reproErr =  (fisheye) ? reproFisheye : reproPinhole; //allowed reprojection error [px] regarding outlier elimination 
	int iterationsCount = 10000;

	// search for outlier (please do not use for fisheye cameras as solvePnP expect central perspecive rather than fisheye
	std::vector<cv::Point3f> object_points_ransac;
	std::vector<cv::Point2f> image_points_real_ransac;

	 // find matching indices in both vectors of matching points
	// note: solve PnP considers central perspective images. Using fisheye will lead to many matches to be detected as outliers, set reproError to a high value to get most matches

	
	cv::solvePnPRansac(in_matched_object_points, in_matched_image_points_real, camera_matrix, dist_coeffs, rvec, tvec, true, iterationsCount = iterationsCount, solvePnPRansac_reproErr, 0.9999, inliers, CV_ITERATIVE);
	for (int i = 0; i < in_matched_object_points.size(); i++) {
		if (std::find(inliers.begin(), inliers.end(), i) != inliers.end()) {
			image_points_real_ransac.push_back(in_matched_image_points_real[i]);
			object_points_ransac.push_back(in_matched_object_points[i]);
		}
	}
	mLogFile->append(TAG + "total matched (object_points_ransac/image_points_real_ransac): " + std::to_string(object_points_ransac.size()) + "/" + std::to_string(image_points_real_ransac.size()));
	

	// -- run bundle adjustment, calibration and determination of position 
	// call re-implementation of calibrateCamera valid for 3D patterns,
	// use initial ideal camera_matrix as approximation for intrinsic parameters; use projection center and extrinsics from data_manager as approximations for extrinsics
	// run Levenberg-Marquard adjustment to minimize reprojection error;
	// converge after 300 iterations or TermCriteria::COUNT + EPS
	// result: camera matrix, dist_coeefs, optimised position and orientation - valid for current application case
	cv::Mat perViewErrors;

	cv::TermCriteria termCrit = cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS), 300, DBL_EPSILON);	// criterium for termination/ converge, max number of iterations: 300
	//criteria = TermCriteria(TermCriteria::EPS+TermCriteria::COUNT, 20, FLT_EPSILON) 


	// define calibration flags + init repro_error
	float repro_error = -1.0f;

	// 3. performance
	if (in_flag == FIXED_EO_IO) {
		mLogFile->append(TAG + "IO fixed, EO fixed");
		return -1;
	}
	// fix entire IOR
	if (in_flag == CALC_EO) {
		mLogFile->append(TAG + "IO fixed, EO optimisation via solvePnPRefineLM");
		cv::solvePnPRefineLM(object_points_ransac, image_points_real_ransac, camera_matrix, dist_coeffs, rvec, tvec, termCrit);
	}
	else {
		mLogFile->append(TAG + "IO + EO optimisation");
		std::vector<std::vector<cv::Point3f>> object_points_ransac_vector;	// OCV required data structure
		std::vector<std::vector<cv::Point2f>> image_points_real_ransac_vector;	// OCV requred data structure
		object_points_ransac_vector.push_back(object_points_ransac);
		image_points_real_ransac_vector.push_back(image_points_real_ransac);

		if (!fisheye) {
			int flagsCalib = CV_CALIB_USE_INTRINSIC_GUESS;// +CV_CALIB_FIX_ASPECT_RATIO; // Test 28.07.2022
			repro_error = cv::calibrateCamera(object_points_ransac_vector, image_points_real_ransac_vector, true_image.size(), camera_matrix, dist_coeffs, rvec, tvec, stdDev_In, stdDev_Ext, perViewErrors, flagsCalib, termCrit);
		}
		else {
			// Todo find out how to use extrinsics here -> perhaps simply put to vector?
			// https://gist.github.com/suzumura-ss/db91b901f5a300e6b9949cf5e012278e
			std::vector<cv::Mat> rvecs, tvecs;
			int flagsCalib = cv::fisheye::CALIB_USE_INTRINSIC_GUESS + cv::fisheye::CALIB_FIX_SKEW + cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_FIX_K4 + cv::fisheye::CALIB_FIX_K3 + cv::fisheye::CALIB_FIX_K2 + cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
			repro_error = cv::fisheye::calibrate(object_points_ransac_vector, image_points_real_ransac_vector, true_image.size(), camera_matrix, dist_coeffs, rvecs, tvecs, flagsCalib, termCrit);
		}
	}


	// Compute repro error & covariance matrix to estimate standard deviations [things done by OCV are weird, do it yourself!]: https://stackoverflow.com/a/20911906
	std::vector<cv::Point2f> image_points_repro;
	cv::Mat jacobian;

	if (fisheye) {
		cv::fisheye::projectPoints(object_points_ransac, image_points_repro, rvec, tvec, camera_matrix, dist_coeffs, 0, jacobian);

	} else {
		cv::projectPoints(object_points_ransac, rvec, tvec, camera_matrix, dist_coeffs, image_points_repro, jacobian);	
		// central perspective / undistort image only!
		// Compute standard deviation for extrinsics and intrinsics [no need for dist_coeffs as we undistort the images immediately and stdDevs of distortion parameters are difficult to interpret]
		cv::Mat sigma_extr = cv::Mat(jacobian.t() * jacobian, cv::Rect(0, 0, 6, 6)).inv(); // jacobian matrix of derivatives of image points with respect to components of the rotation vector, translation vector, focal lengths, coordinates of the principal point and the distortion coefficients
		cv::Mat sigma_intr = cv::Mat(jacobian.t() * jacobian, cv::Rect(6, 6, 9, 9)).inv();
		sqrt(sigma_extr.diag(), stdDev_Ext);
		sqrt(sigma_intr.diag(), stdDev_In);
	}

	

	// Compute rmse
	float mean_error = 0.0f;
	for (int i = 0; i < image_points_real_ransac.size(); i++) {
		float error = cv::norm(cv::Mat(image_points_real_ransac.at(i)), cv::Mat(image_points_repro.at(i)), cv::NORM_L2, cv::noArray());
		mean_error += error;
	}
	repro_error = mean_error / object_points_ransac.size();  // mean_error / image_points_real_ransac.size();
	mReproError4JSON = repro_error;	// copy repro error for json output

	std::cout << "My repro error " << repro_error << std::endl;

	// data_conversion if necessary, convert data (64FC3 to 64FC1)
	if (tvec.channels() == 3 || rvec.channels() == 3) {
		std::vector<cv::Mat> splitMats_r, splitMats_t;
		cv::Mat rvec_1ch, tvec_1ch;

		cv::split(rvec, splitMats_r); cv::split(tvec, splitMats_t);
		rvec_1ch.push_back(splitMats_r[0]);	rvec_1ch.push_back(splitMats_r[1]);	rvec_1ch.push_back(splitMats_r[2]);
		tvec_1ch.push_back(splitMats_t[0]);	tvec_1ch.push_back(splitMats_t[1]);	tvec_1ch.push_back(splitMats_t[2]);

		// save original values from solvepnp implemented in calibrate camera
		rvec_1ch.copyTo(rvec);
		tvec_1ch.copyTo(tvec);
	}
	// end spatial resection
	
	draw_valid_matches_after_back_projection(true_image, image_points_real_ransac, image_points_repro);
	// ----------------- end output section ---------------------- //

	return repro_error;
}








/**
 * @fn	void Matching::waterlineProjection(std::vector<cv::Point2d>& in_wl_pts_2D, std::vector<Vek3d>& synth_pts_3D, cv::Mat& in_image_4_color, cv::Mat& camera_matrix,cv::Mat& dist_coeffs, cv::Mat& rvec_cc_orig_copy, cv::Mat& tvec_cc_orig_copy, double shift_x, double shift_y)
 *
 * @brief	Waterline projection.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	in_wl_pts_2D	 	[input] detected water line points (2D)
 * @param [in,out]	synth_pts_3D	 	[input,output] 3D object points inside frustum, correspond to 2D points of virtual rendered image
 * @param [in,out]	in_image_4_color 	[input,output] input image of size of smartphone camera image that is used for point cloud colouration
 * @param [in,out]	camera_matrix		[input,output] camera matrix corresponding to (real) smartphone image
 * @param [in,out]	dist_coeffs			[input,output] distortion coefficients corresponding to (real) smartphone image
 * @param [in,out]	rvec_cc_orig_copy	[input,output] rotation vector/matrix
 * @param [in,out]	tvec_cc_orig_copy	[input,output] translation vector
 * @param 		  	shift_vector_x   	[input] shift vector x coordinate, shift re-aligned point clouds with large coordinates (UTM) back to roots
 * @param 		  	shift_vector_y   	[input] shift vector y coordinate, shift re-aligne point clouds with large coordinates (UTM) back to roots
 */

void Matching::waterlineProjection(std::vector<cv::Point2d>& in_wl_pts_2D, std::vector<Vek3d>& synth_pts_3D, cv::Mat& in_image_4_color, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec_cc_orig_copy, cv::Mat& tvec_cc_orig_copy, double shift_x, double shift_y, double shift_z, bool export_pcl) {

	
	
	// --- init variables ---
	// remove points that doesn´t fit distance from plane for further processing
	// remove points that may be highly affacted by distortion (which could not determined precisely)
	double max_distance_from_line = 0.025; // m
	double tolerance_distortion = 5; // px

	// logger for stats in this function
	std::stringstream log_statistics;
	// undistort water line points and push back into image coordinate system
	std::vector<cv::Point2d> wl_pts_2D_undistort_normalized_coordinates;
	std::vector<cv::Point2d> wl_pts_2D_undistort_image_coordinates;
	std::vector<cv::Point2d> water_line_points_reduced_by_dist; // check distance water line undistort -> water line distort, remove points in regions of uncertainity

	// ----- 3D projection  using OCV model ----- //
	// for opencv project points and application of collinearity equation using cv model! 
	Modell_OCV modell_ocv = Modell_OCV(mLogFile);
	std::vector<cv::Vec3b> point_cloud_color;
	std::vector<cv::Point2d> image_coordinates_color;

	// Computes the ideal point coordinates from the observed point coordinates. (CV Doc)
	// where undistort() is an approximate iterative algorithm that estimates the normalized original point coordinates out of the normalized distorted point coordinates (“normalized” means that the coordinates do not depend on the camera matrix).
	// --> coordinates must be converted in image coordinates
	//		(u,v) is the input point, (u', v') is the output point
	//		camera_matrix=[fx 0 cx; 0 fy cy; 0 0 1]
	//		P=[fx' 0 cx' tx; 0 fy' cy' ty; 0 0 1 tz]
	//		x" = (u - cx)/fx
	//		y" = (v - cy)/fy
	//		(x',y') = undistort(x",y", dist_coeffs)
	//		[X, Y, W]T = R*[x' y' 1]T
	//		x = X / W, y = Y / W
	//		// only performed if P=[fx' 0 cx' [tx]; 0 fy' cy' [ty]; 0 0 1 [tz]] is specified // here not specified thus must be converted manually afterwards
	//		u' = x*fx' + cx'
	//		v' = y*fy' + cy',

	if (in_wl_pts_2D.size() != 0) {
		// undist 2D image points of water line
		cv::undistortPoints(in_wl_pts_2D, wl_pts_2D_undistort_normalized_coordinates, camera_matrix, dist_coeffs);

			// conversion image coordinates
			for (cv::Point2d p : wl_pts_2D_undistort_normalized_coordinates) {
				wl_pts_2D_undistort_image_coordinates.push_back(cv::Point2d(
					camera_matrix.at<double>(0, 0) * p.x + camera_matrix.at<double>(0, 2),
					camera_matrix.at<double>(1, 1) * p.y + camera_matrix.at<double>(1, 2)));
			}

			// compute distance between distorted image coordinates and undistorted image coordinates, push back only coordinates in "save" distance [tolerance_distortion]
			for (int i = 0; i < in_wl_pts_2D.size(); i++) {
				cv::Point2d p_distorted = in_wl_pts_2D.at(i);
				cv::Point2d p_undistorted = wl_pts_2D_undistort_image_coordinates.at(i);
				double distance_undistortion = sqrt(sq(p_distorted.x - p_undistorted.x) + sq(p_distorted.y - p_undistorted.y));

				if (distance_undistortion < tolerance_distortion)
					water_line_points_reduced_by_dist.push_back(p_undistorted); // new push_back undistorted line
			}
			mLogFile->append(TAG + "count reduced 2D waterline points regarding image distortion (redu/orig): " + std::to_string(water_line_points_reduced_by_dist.size()) + "/" + std::to_string(in_wl_pts_2D.size()));
	} else {
		mLogFile->append(TAG + "no waterline provided, will only color point cloud");
	}
	

	// convert synth_pts_3D to cv::Point3d
	std::vector<cv::Point3d> synth_pts_3D_cv;
	for (Vek3d vec : synth_pts_3D)
		synth_pts_3D_cv.push_back(cv::Point3d(vec.x(), vec.y(), vec.z()));

	// get colors from real image for point cloud colorization (nice to have for validation) and receive 3D depth coordinates for water line points using
	// kdtree.knnSearch to look for nearest neighbour image pixel water line <-> virtual image. take corresponding 3D values respectively
	std::vector<cv::Point3d> projected_waterline_3D_OCV = modell_ocv.getColorFor(synth_pts_3D_cv, in_image_4_color, point_cloud_color, image_coordinates_color, 1.0, camera_matrix, dist_coeffs, rvec_cc_orig_copy, tvec_cc_orig_copy, water_line_points_reduced_by_dist);


	// print point cloud
	if (export_pcl) {
		mLogFile->append(TAG + "---- export 3D point cloud ----");
		modell_ocv.export_point_cloud_recolored(mWorkingDirectory, mDataManager->get_name_working_directory(), synth_pts_3D_cv, point_cloud_color, image_coordinates_color, shift_x, shift_y, shift_z);
	}

	// Water line analysis & statistics
	if (projected_waterline_3D_OCV.size() != 0) {
		mLogFile->append(TAG + "---- start water line projection ----");
		log_statistics << "OCV, projected " << projected_waterline_3D_OCV.size() << " water line points into object space" << std::endl << std::endl;

		// 3D plane fitting using SVD
		std::vector<Eigen::Vector3d> points;

		for (cv::Point3d& p : projected_waterline_3D_OCV)
			points.push_back(Eigen::Vector3d(p.x, p.y, p.z));

		std::pair<Eigen::Vector3d, Eigen::Vector3d> result = best_plane_from_points(points);

		Eigen::Vector3d centroid = result.first;
		Eigen::Vector3d normal = result.second;

		// get positive normal
		normal *= (-1);

		// infos about svd (https://www.ltu.se/cms_fs/1.51590!/svd-fitting.pdf)
		log_statistics << "fit 3D plane using SVD" << endl;
		log_statistics << "centroid of plane: (" << centroid.x() << "," << centroid.y() << "," << centroid.z() << ")" << std::endl;
		log_statistics << "normal of plane: (" << normal.x() << "," << normal.y() << "," << normal.z() << ")" << std::endl << std::endl;


		// remove points that doesn´t fit distance from plane for furthe processing
		std::vector<cv::Point3d> optimized_waterline;
		std::vector<float> distances;

		double mean_sq_plane = 0.0f;
		double root_mean_sq_plane = 0.0f;

		// calc distance 3D water level point <-> fitted 3D plane
		std::vector<int> pt_ids;
		int pt_id = 0;
		for (Eigen::Vector3d vec : points) {
			float dist = normal.dot(vec - centroid);
			mean_sq_plane += sq(dist);

			if (dist < max_distance_from_line) {
				optimized_waterline.push_back(cv::Point3d(vec.x(), vec.y(), vec.z()));
				distances.push_back(dist);
				pt_ids.push_back(pt_id);
			}
			pt_id++;
		}

		mean_sq_plane /= points.size(); // calc mean_sq
		root_mean_sq_plane = sqrt(mean_sq_plane); //calc root_mean_sq

		// ---- statistics ---- //
		std::setprecision(6);
		log_statistics << std::fixed << "---- statistics for waterline calculation, plane fit ----" << std::endl;
		log_statistics << std::fixed << "sq_mean: " << mean_sq_plane << std::endl;
		log_statistics << std::fixed << "rms error: " << root_mean_sq_plane << std::endl << std::endl;
		log_statistics << std::fixed << "count water level points that fullfil criterion: dist_plane < " << max_distance_from_line << ": " << optimized_waterline.size() << std::endl << std::endl;


		// use only points with distance < threshold (currently 0.025 m)
		projected_waterline_3D_OCV.clear(); // clear old projected points
		projected_waterline_3D_OCV = optimized_waterline; // put in optimized water line


		// infos calculation statistics: https://www.frustfrei-lernen.de/mathematik/standardabweichung-berechnen.html
		double mean_z = 0.0, variance_z = 0.0, stdev_z = 0.0;
		double mean_x = 0.0; // for levelling/ shifting to cs origin
		double median_z = 0.0; // for median calc

		std::ofstream myfile;
		std::ofstream myfile_ptids;
		myfile.open(mWorkingDirectory + "waterlinepoints_projected.txt");
		myfile_ptids.open(mWorkingDirectory + "waterlinepoints_projected_pt_IDs.txt");

		// check if point id list has same size as projected water line 3D
		if (projected_waterline_3D_OCV.size() != pt_ids.size()){
			log_statistics << "size of waterline point id list and water line points projected list does not match! Retun." << std::endl << std::endl;
			return;
		}
		else {
			log_statistics << "will save water line points and point ids: " << projected_waterline_3D_OCV.size() << "/" << pt_ids.size() << std::endl << std::endl;

		}


		for (int i = 0; i < projected_waterline_3D_OCV.size(); i++) {
			myfile_ptids << std::fixed << pt_ids[i] << "\n";

			myfile << std::fixed << std::setprecision(4)
				<< (projected_waterline_3D_OCV)[i].x + shift_x << ","
				<< (projected_waterline_3D_OCV)[i].y + shift_y << ","
				<< (projected_waterline_3D_OCV)[i].z + shift_z << "\n";

			mean_z += (projected_waterline_3D_OCV)[i].z;
			mean_x += (projected_waterline_3D_OCV)[i].x;

		}
		mean_z /= projected_waterline_3D_OCV.size(); // calc mean_z
		mean_x /= projected_waterline_3D_OCV.size(); // calc mean_x

		for (int i = 0; i < projected_waterline_3D_OCV.size(); i++) {
			variance_z += sq((projected_waterline_3D_OCV)[i].z - mean_z); //subtr mean
		}

		// calc variance s² 
		variance_z /= projected_waterline_3D_OCV.size();
		// calc standard derviation
		stdev_z = sqrt(variance_z);

		myfile_ptids.close();
		myfile.close();

		

		// project points on xz plane (y = 0)
		// get center along x axis, define this point as water level that has to be measured
		// shift water line before orthogonal projection that water level point x is in image center
		// rotate xz plane arround observed water level that line becomes parallel zu xy-plane (levelling)
		std::vector<cv::Point2d> optimized_water_points_xz;
		for (cv::Point3d &p : optimized_waterline)
			optimized_water_points_xz.push_back(cv::Point2d(p.x - mean_x, p.z));// -mean_z)); //shift to origin by x only


		// fit 2D water line using least squares line fit (cv::fitLine) with standard parameters
		cv::Vec4f lineFit;
		cv::fitLine(optimized_water_points_xz, lineFit, CV_DIST_L12, 0, 0.01, 0.01);

		// get line equation from vector
		double vx = lineFit(0);
		double vy = lineFit(1);
		double x = lineFit(2);
		double y = lineFit(3);

		cv::Point2d p1 = cv::Point2d(x, y);
		cv::Point2d p2 = cv::Point2d(vx * 5 + x, vy * 5 + y); //end point for line equation

		double theta = atan2((p2.y - p1.y), (p2.x - p1.x));
		const double PI = 3.14159265358979323846;
		log_statistics << "fit water line xz 2d: " << vx << "," << vy << "," << x << "," << y << ", size: " << optimized_water_points_xz.size() << std::endl;
		log_statistics << "angle between line and x-axis: " << theta << ", (degrees: " << theta * 180 / PI << ")" << std::endl << std::endl;

		// apply rotation (see https://de.wikipedia.org/wiki/Drehmatrix)
		std::vector < cv::Point2d > optimized_water_points_xz_leveled;

		// calc statistics for leveled waterline 
		double mean_z_leveled = 0.0, variance_z_leveled = 0.0, stdev_z_leveled = 0.0;

		// rotation about negative z-axis
		for (cv::Point2d p : optimized_water_points_xz) {
			double x = p.x * cos(theta) - p.y * -sin(theta);
			double y = p.x * -sin(theta) + p.y * cos(theta);
			optimized_water_points_xz_leveled.push_back(cv::Point2d(x, y));
			mean_z_leveled += y;
		}

		// calc mean_z_leveled
		mean_z_leveled /= optimized_water_points_xz_leveled.size();

		for (cv::Point2d p : optimized_water_points_xz_leveled) {
			variance_z_leveled += sq(p.y - mean_z_leveled);
		}
		// calc variance s² 
		variance_z_leveled /= optimized_water_points_xz_leveled.size();

		// calc standard derviation
		stdev_z_leveled = sqrt(variance_z_leveled);

		// calc median 
		std::vector<double> z_values;

		//push z values into vector
		for (int i = 0; i < projected_waterline_3D_OCV.size(); i++) {
			z_values.push_back((projected_waterline_3D_OCV)[i].z);
		}


		if (z_values.size() % 2 == 0) {
			//even
			const auto median_it1 = z_values.begin() + z_values.size() / 2 - 1;
			const auto median_it2 = z_values.begin() + z_values.size() / 2;
			std::nth_element(z_values.begin(), median_it1, z_values.end()); // e1
			std::nth_element(z_values.begin(), median_it2, z_values.end()); // e2
			auto median = (z_values.size() % 2 == 0) ? (*median_it1 + *median_it2) / 2 : *median_it2;
			median_z = median;
		}

		else {
			// odd
			const auto median_it = z_values.begin() + z_values.size() / 2;
			std::nth_element(z_values.begin(), median_it, z_values.end());
			auto median = *median_it;
			median_z = median;
		}


		log_statistics << "---- statistics for waterline calculation without terrain compensation ---- " << std::endl;
		log_statistics << "mean_z: " << mean_z << std::endl;
		log_statistics << "median_z: " << median_z << std::endl;
		log_statistics << "variance_z: " << variance_z << std::endl;
		log_statistics << "stdev_z.n: " << stdev_z << std::endl;
		log_statistics << "stdev_z.n 3x: " << stdev_z*3.0 << std::endl << std::endl;

		// projected water line on xz-plane, levelling water line
		log_statistics << "---- statistics for waterline calculation with terrain compensation/ levelling ----" << std::endl;
		log_statistics << "mean_z_leveled: " << mean_z_leveled << std::endl;
		log_statistics << "variance_z_leveled: " << variance_z_leveled << std::endl;
		log_statistics << "stdev_z_leveled.n: " << stdev_z_leveled << std::endl;
		log_statistics << "stdev_z_leveled.n 3x: " << stdev_z_leveled*3.0 << std::endl << std::endl;

		//console output
		std::string line;
		while (std::getline(log_statistics, line)) {
			mLogFile->append(TAG + line);
		}

		// ---- write results to json file for deliery android ----
		nlohmann::json json_result = {
			{ "mean_z_leveled", mean_z },
			{ "variance_z_leveled", variance_z_leveled },
			{ "stdev_z_leveled_n", stdev_z_leveled },
			{ "stdev_z_leveled_n3", stdev_z_leveled*3.0 },
			{ "rms_err_plane_fit", root_mean_sq_plane },
			{ "water_level_size", optimized_water_points_xz.size() },
			{ "camera_repro_error", mReproError4JSON },
		};

		// write prettified JSON to another file
		std::string json_file_name = mDataManager->get_uuid(); // get uuid for json_name (if no uuid is provided, json name == no_uuid

		std::ofstream o(mWorkingDirectory + json_file_name + ".json");
		o << std::setw(4) << json_result << std::endl; //sets width of number n

		mLogFile->append(TAG + "have written results to JSON");

		std::ostringstream line_4_results;
		line_4_results << mDataManager->get_filename_true_image() << "; median;" << median_z << "; mean_z; " << mean_z << "; 3stdev_z; " << stdev_z*3.0 << "; mean_z_leveled; " << mean_z_leveled << "; 3stdev_z_leveled; " << stdev_z_leveled *3.0
			<< ";repro_error;" << mReproError4JSON << ";inl_dist_cl_wl_perc;" << std::to_string(inliers_per_importance_cell_percent) << ";inl_dist_cl_wl_no;" << std::to_string(point_counter_importance) << ";total_inliers;" << std::to_string(no_inliers_4_stat)
			<< ";f;" << camera_matrix.at<double>(0, 0) << ";cx;" << camera_matrix.at<double>(0, 2) << ";cy;" << camera_matrix.at<double>(1, 2) << ";a;" << dist_coeffs.at<double>(0, 0) << ";" << dist_coeffs.at<double>(1, 0) << ";" << dist_coeffs.at<double>(2, 0)
			<< ";b;" << dist_coeffs.at<double>(3, 0) << ";" << dist_coeffs.at<double>(4, 0) << ";c;" << dist_coeffs.at<double>(5, 0) << ";" << dist_coeffs.at<double>(6, 0);
		Utils::append_line_to_file(mDataManager->get_path_working_directory() + "\\result.txt", line_4_results.str());


		// ----------------------------------------------------------------------------------------
		// 22.10.2020 backprojection 3D water line into camera image (master)
		// projected_waterline_3D_OCV_ptr = 3D water line coordinates
		std::vector<cv::Point2d> imagePoints;
		cv::projectPoints(projected_waterline_3D_OCV, rvec_cc_orig_copy, tvec_cc_orig_copy, camera_matrix, dist_coeffs, imagePoints);
		
		cv::Mat copy_masterimage = in_image_4_color.clone();
		

		// draw oroginal water line  on canvas
		for (cv::Point2d p : water_line_points_reduced_by_dist) {
			cv::circle(copy_masterimage, p, 5, cv::Scalar(255, 255, 0), -1); // circle water line on canvas


		}
		// draw reprojected water line on canvas
		for (cv::Point2d p : imagePoints) {
			cv::circle(copy_masterimage, p, 4, cv::Scalar(255, 0, 255), -1); // circle water line on canvas
		}

		cv::imwrite(mWorkingDirectory + "reprojected_waterline.png", copy_masterimage);
		mLogFile->append(TAG + "visualisation: apply cv::projectPoints, draw water line reprojected");
		// ----------------------------------------------------------------------------------------

	}

	else if (in_wl_pts_2D.size() != 0) { // only append to text file if 2D water line was given but it could not be intersected with 3D data; otherwise tool was only run to color point cloud
		mLogFile->append(TAG + "no water level detected");
		std::ostringstream line_4_results;
		line_4_results << mDataManager->get_filename_true_image() + ";_;no_level";
		Utils::append_line_to_file(mDataManager->get_path_working_directory() + "\\result.txt", line_4_results.str());
	}

	

}



//////////////////////////////////// OUTPUT FUNCTIONS //////////////////////////////////////////////////////////////////////////

 /**
  * @fn	void Matching::draw_valid_matches_after_back_projection(cv::Mat&amp; in_canvas, std::vector&lt;cv::Point2f&gt;&amp; in_image_points_real_ransac, std::vector&lt;cv::Point2f&gt;&amp; in_matched_object_points_ransac_projected)
  *
  * @brief	Draw valid matches after back projection. Visualisation of valid matches determined by project object points into image space using calculated intrinsic and extrinsics
  *
  * @author	Mela
  *
  * @date	25.04.2018
  *
  * @param [in,out]	in_canvas								 	cv::Mat canvas image (will be overwritten)
  * @param [in,out]	in_image_points_real_ransac				 	The in image points real ransac.
  * @param [in,out]	in_matched_object_points_ransac_projected	The in matched object points ransac projected.
  */

void Matching::draw_valid_matches_after_back_projection(cv::Mat& image, std::vector<cv::Point2f>& in_image_points_real_ransac, std::vector<cv::Point2f>& in_matched_object_points_ransac_projected) {

	cv::Mat in_canvas = image.clone();
	// make image darker for better contrast to matches
	// Do the operation new_image(i,j) = alpha*image(i,j) + beta
	cv::Mat in_canvas_lower_brightness_100;
	in_canvas.convertTo(in_canvas_lower_brightness_100, -1, 1, -100); //decrease the brightness by 100
	in_canvas_lower_brightness_100.copyTo(in_canvas);

	// check points , keep aspect_ration fx=fy

	// draw green/red circles for re-projected image points (inside/outside)
	// define colors
	const double thresh_good = 1.0; // repro error, should be under 1.0 px
	const double thresh_mid = 3.0;
	const double thresh_critic = 5.0;
	const double thresh_high = 7.0;

	for (uint i = 0; i < in_matched_object_points_ransac_projected.size(); i++) {
		double eukli = eucl_Distance(in_matched_object_points_ransac_projected.at(i), in_image_points_real_ransac.at(i));

		if (eukli <= thresh_good)
			cv::circle(in_canvas, in_image_points_real_ransac.at(i), 3, cv::Scalar(0, 255, 0), 1, -1);
		else if (eukli <= thresh_mid)
			cv::circle(in_canvas, in_image_points_real_ransac.at(i), 3, cv::Scalar(0, 255, 128), 1, -1);
		else if (eukli <= thresh_critic)
			cv::circle(in_canvas, in_image_points_real_ransac.at(i), 3, cv::Scalar(0, 255, 255), 1, -1);
		else if (eukli <= thresh_high)
			cv::circle(in_canvas, in_image_points_real_ransac.at(i), 3, cv::Scalar(0, 128, 255), 1, -1);
		else
			cv::circle(in_canvas, in_image_points_real_ransac.at(i), 3, cv::Scalar(0, 0, 255), 1, -1);

		// draw arrow from old to new location
		cv::arrowedLine(in_canvas, in_image_points_real_ransac.at(i), in_matched_object_points_ransac_projected.at(i), cv::Scalar(255, 255, 0));

	}
	cv::imwrite(mWorkingDirectory + "mask_valid_pts.png", in_canvas);
	mLogFile->append(TAG + "visualisation: apply cv::projectPoints, calc distances image position <-> re-projected image positions");


}





// check distribution of inliers for spatial intersection with camera parameter estimation
// real_image_size defines size of grid to check
// inliers must corrspond to inlieres to be checked
// raster_size defines raster width that must be the same for columns and rows and ist standard = 4 (results in 4x4 raster)
// file_name for save image to working directory

/**
 * @fn	void Matching::draw_inliers_distribution(cv::Mat& real_image, std::vector<cv::Point2d>& inliers, std::vector<cv::Point2d>& waterline_points, int raster_size = 4, std::string file_name = "matching_distribution")
 *
 * @brief	Draw inliers distribution.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	real_image			The real image.
 * @param [in,out]	inliers				The inliers.
 * @param [in,out]	waterline_points	The waterline points.
 * @param 		  	raster_size			(Optional) Size of the raster.
 * @param 		  	file_name			(Optional) Filename of the file.
 */

void Matching::draw_inliers_distribution(cv::Mat& real_image, std::vector<cv::Point2d>& inliers, std::vector<cv::Point2d>& waterline_points, int raster_size = 4, std::string file_name = "matching_distribution") {
	std::vector<cv::Rect> list_of_rects;
	int cell_width = real_image.cols / raster_size;
	int cell_height = real_image.rows / raster_size;

	int cell_origin_x = 0;
	int cell_origin_y = 0;

	for (int x = 0; x < raster_size; x++) {
		for (int y = 0; y < raster_size; y++) {

			cell_origin_y = cell_height * y;
			cell_origin_x = cell_width * x;

			list_of_rects.push_back(cv::Rect(cell_origin_x, cell_origin_y, cell_width, cell_height));
		}
		//std::cout << "x: " << std::to_string(cell_origin_x) + ",y: " << std::to_string(cell_origin_y) << std::endl;
	}

	cv::Mat copy_real_image, color_mask_rgb;
	real_image.copyTo(copy_real_image);

	cv::Mat color_mask = cv::Mat::zeros(real_image.size(), CV_8UC1);

	// check which cell contains image center
	cv::Point2d image_center = cv::Point2d(real_image.cols / 2, real_image.rows / 2);


	//std::cout << "Region of importance: x,y: " << std::to_string(region_of_importance.x)  << "," << std::to_string(region_of_importance.y) << "w/h: " << std::to_string(region_of_importance.width) << "," << std::to_string(region_of_importance.height) << std::endl;

	// create bounding box around water line points for region of importance [if waterline points are given]
	// draw points onto blank image plane
	cv::Mat water_line_points_canvas = cv::Mat::zeros(real_image.size(), CV_8UC1);
	for (cv::Point2d p : waterline_points) {
		cv::circle(water_line_points_canvas, p, 3, 255, -1);
		cv::circle(copy_real_image, p, 1, cv::Scalar(255, 255, 0), -1); // circle water line on canvas
	}

	// draw inliers on canvas
	for (cv::Point2d p : inliers) {
		cv::circle(copy_real_image, p, 3, cv::Scalar(255, 0, 255), -1); // circle water line on canvas
	}

	cv::Mat threshold_output;
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	/// Detect edges using Threshold
	threshold(water_line_points_canvas, threshold_output, 200, 255, CV_THRESH_BINARY);

	/// Find contours
	findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	/// Find the convex hull object for each contour
	//std::vector<std::vector<cv::Point> >hull(contours.size());
	//for (int i = 0; i < contours.size(); i++) {
	//	cv::convexHull(cv::Mat(contours[i]), hull[i], false);
	//}

	/// Approximate contours to polygons + get bounding rects and circles
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());
	std::vector<cv::Rect> boundRect(contours.size());

	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 3, true);
		boundRect[i] = boundingRect(cv::Mat(contours_poly[i]));
	}
	//cv::rectangle(copy_real_image, boundRect[0], cv::Scalar(0, 255, 255), 2, 8, 0);



	/// Draw contours + hull results
	// enlarge rectangle and define region of importance --> 1/4 of image size
	cv::Point inflationPoint(-real_image.size().width / 8, -real_image.size().height / 8);
	cv::Size inflationSize(real_image.size().width / 4, real_image.size().height / 4);
	boundRect[0] += inflationPoint;
	boundRect[0] += inflationSize;


	// 2/3 of image's width and height for region of importance
	cv::Rect region_of_importance = boundRect[0]; // cv::Rect(image_center.x / 4, image_center.y / 4, real_image.cols * 3 / 4, real_image.rows * 3 / 4);
	//cv::rectangle(copy_real_image, region_of_importance, cv::Scalar(0, 0, 255), 5);
	cv::rectangle(copy_real_image, region_of_importance, cv::Scalar(0, 255, 255), 2, 8, 0);

	int counter = 0;
	std::vector<double> inliers_per_cell_percent;
	std::vector<int> inliers_per_cell;

	for (cv::Rect r : list_of_rects) {
		cv::rectangle(copy_real_image, r, cv::Scalar(0, 0, 255));
		cv::putText(copy_real_image, std::to_string(counter), (r.br() + r.tl())*0.5, CV_FONT_NORMAL, 5, cv::Scalar(0, 0, 255));

		// check if inlier is inside of rect
		double point_counter = 0;
		for (cv::Point2d p : inliers) {
			if (r.contains(p))
				point_counter++;
		}
		inliers_per_cell_percent.push_back(point_counter / inliers.size() * 100);
		inliers_per_cell.push_back(point_counter);
		cv::rectangle(color_mask, r, point_counter / inliers.size() * 255, -1);

		counter++;

	}

	// check point distribution
	std::string well_dist = "yes";
	bool well_dist_bool = true;
	mLogFile->append(TAG + "list of feature points per cell [%]: ");
	for (int i = 0; i < inliers_per_cell_percent.size(); i++) {
		mLogFile->append(TAG + std::to_string(i) + ": " + std::to_string(inliers_per_cell_percent[i]) + ", No: " + std::to_string(inliers_per_cell[i]) + "/" + std::to_string(inliers.size()));
		if (inliers_per_cell_percent[i] == 0.0) {
			well_dist = "no";
			well_dist_bool = false;
			break;
		}
	}
	mDataManager->set_well_distributed_object_points_image_space(well_dist_bool); // in case that there are no assigned object points in image space, set well-distribution flag to false
	mLogFile->append(TAG + "have well-distributed 2D-3D point pairs? (image space): " + well_dist);
	
	point_counter_importance = 0;
	for (cv::Point2d p : inliers) {
		if (region_of_importance.contains(p))
			point_counter_importance++;
	}
	inliers_per_importance_cell_percent = point_counter_importance / inliers.size() * 100;
	mLogFile->append(TAG + "points inside region of importance [%]: " + std::to_string(inliers_per_importance_cell_percent) + ", No: " + std::to_string(point_counter_importance) + "/" + std::to_string(inliers.size()));

	// output
	cv::applyColorMap(color_mask, color_mask_rgb, 5);
	cv::add(color_mask_rgb, copy_real_image, copy_real_image);
	cv::imwrite(mWorkingDirectory + file_name + ".jpg", copy_real_image);

	no_inliers_4_stat = inliers.size();


}




// output function, draw matches re-implementation

/**
 * @fn	void Matching::draw_matches_reimpl_MK(cv::Mat& in_real_image, cv::Mat& in_synth_image, std::vector <Vek2d>& in_real_matches_draw, std::vector <Vek2d>& in_synth_matches_draw, std::string fileName)
 *
 * @brief	Draw matches reimpl mk.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	in_real_image		 	The in real image.
 * @param [in,out]	in_synth_image		 	The in synth image.
 * @param [in,out]	in_real_matches_draw 	The in real matches draw.
 * @param [in,out]	in_synth_matches_draw	The in synth matches draw.
 * @param 		  	fileName			 	Filename of the file.
 */

void Matching::draw_matches_reimpl_MK(cv::Mat& in_real_image, cv::Mat& in_synth_image, std::vector <Vek2d>& in_real_matches_draw, std::vector <Vek2d>& in_synth_matches_draw, std::string fileName) {

	//cv::Mat synthImage = mDataManager->get_synth_image();
	//cv::Mat realImage = mDataManager->get_real_image();

	// ---- start output, draw inlier matches ---- //


	cv::Mat matchesImage = cv::Mat(in_synth_image.rows > in_real_image.rows ? in_synth_image.rows : in_real_image.rows, in_synth_image.cols + in_real_image.cols, in_synth_image.type(), cv::Scalar(0,0,0));


	mLogFile->append(TAG + "visualisation: draw matches");

	for (int i = 0; i < matchesImage.rows; i++) {
		for (int j = 0; j < matchesImage.cols; j++) {
			if (j < in_real_image.cols && i < in_real_image.rows)
				matchesImage.at<cv::Vec3b>(i, j) = in_real_image.at<cv::Vec3b>(i, j);
		}
	}

	for (int i = 0; i < matchesImage.rows; i++) {
		for (int j = in_real_image.cols; j < matchesImage.cols - 1; j++) {
			if (j - in_real_image.cols < in_synth_image.cols && i < in_synth_image.rows) {
				matchesImage.at<cv::Vec3b>(i, j) = in_synth_image.at<cv::Vec3b>(i, j - in_real_image.cols);
			}
		}
	}

	for (int i = 0; i < (int)in_real_matches_draw.size(); i++) {
		//draw a line between keypoints with random color
		cv::Scalar clr(rand() % 255, rand() % 255, rand() % 255);

		// NEW: draw a line between keypoints with red color
		clr = cv::Scalar(0, 0, 255);

		//query image is the real frame
		cv::Point2d point_real = cv::Point2d(in_real_matches_draw[i].x(), in_real_matches_draw[i].y());

		//train image is the next frame that we want to find matched keypoints, for Visualisation - add width of realImage
		cv::Point2d point_synth = cv::Point2d(in_synth_matches_draw[i].x(), in_synth_matches_draw[i].y());
		point_synth.x += in_real_image.cols;

		//keypoint color for frame 1: RED
		circle(matchesImage, point_real, 3, clr, 1);

		//keypoint color for frame 2: CYAN
		circle(matchesImage, point_synth, 3, clr, 1);

		//connect both points!
		cv::line(matchesImage, point_real, point_synth, clr, 2, 8, 0);
	}

	cv::imwrite(mWorkingDirectory + fileName + ".jpg", matchesImage);
	// ---- end output, draw inlier matches ---- //


}

/**
 * @fn	void Matching::output_enhanced_spatial_resection(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& in_StdDev_IntO, cv::Mat& in_StdDev_ExtO, cv::Mat& in_PerViewErrors, double in_pix_size)
 *
 * @brief	Output enhanced spatial resection.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	in_camera_matrix	The in camera matrix.
 * @param [in,out]	in_dist_coeffs  	The in distance coeffs.
 * @param [in,out]	in_rvec				The in rvec.
 * @param [in,out]	in_tvec				The in tvec.
 * @param [in,out]	in_StdDev_IntO  	The in standard development int o.
 * @param [in,out]	in_StdDev_ExtO  	The in standard development extent o.
 * @param [in,out]	in_PerViewErrors	The in per view errors.
 * @param 		  	in_pix_size			Size of the in pix.
 */

void Matching::output_enhanced_spatial_resection(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& in_StdDev_IntO, cv::Mat& in_StdDev_ExtO, cv::Mat& in_PerViewErrors, double in_pix_size) {

	// ---- start output section ---- //
	// print camera parameters
	mLogFile->append("");
	mLogFile->append(TAG + "---- camera calibration results ----");

	double *camera_matrix_data = (double*)(in_camera_matrix.data);
	// ocv_docu: A(0, 0) = param[0]; A(1, 1) = param[1]; A(0, 2) = param[2]; A(1, 2) = param[3]; std::copy(param + 4, param + 4 + 14, k);
	double *stdDev_In_data = (double*)(in_StdDev_IntO.data);
	mLogFile->append("\n" + TAG + "------------- intrinsics -------------");
	mLogFile->append(TAG + "fx: " + std::to_string(camera_matrix_data[0]) + " [px]/ " + std::to_string(camera_matrix_data[0] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[0]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[0] * in_pix_size), 5);
	mLogFile->append(TAG + "fy: " + std::to_string(camera_matrix_data[4]) + " [px]/ " + std::to_string(camera_matrix_data[4] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[1]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[1] * in_pix_size), 5);
	mLogFile->append(TAG + "cx: " + std::to_string(camera_matrix_data[2]) + " [px]/ " + std::to_string(camera_matrix_data[2] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[2]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[2] * in_pix_size), 5);
	mLogFile->append(TAG + "cy: " + std::to_string(camera_matrix_data[5]) + " [px]/ " + std::to_string(camera_matrix_data[5] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[3]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[3] * in_pix_size), 5);

	//std::cout.precision(8);
	double *dist_coeffs_data = (double*)(in_dist_coeffs.data);
	mLogFile->append("\n" + TAG + "------------- distortion coefficents -------------");
	mLogFile->append(TAG + "k1: " + std::to_string(dist_coeffs_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[4]), 8);
	mLogFile->append(TAG + "k2: " + std::to_string(dist_coeffs_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[5]), 8);
	mLogFile->append(TAG + "p1: " + std::to_string(dist_coeffs_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[6]), 8);
	mLogFile->append(TAG + "p2: " + std::to_string(dist_coeffs_data[3]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[7]), 8);
	mLogFile->append(TAG + "k3: " + std::to_string(dist_coeffs_data[4]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[8]), 8);

	//std::cout.precision(5);
	double *rvec_cc_1ch_data = (double*)(in_rvec.data);
	double *tvec_cc_1ch_data = (double*)(in_tvec.data);
	double *stdDev_Ext_data = (double*)(in_StdDev_ExtO.data);
	mLogFile->append("\n" + TAG + "------------- extrinsics -------------");
	mLogFile->append(TAG + "rotV r0: " + std::to_string(rvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[0]), 5);
	mLogFile->append(TAG + "rotV r1: " + std::to_string(rvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[1]), 5);
	mLogFile->append(TAG + "rotV r2: " + std::to_string(rvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[2]), 5);
	mLogFile->append(TAG + "transV t0: " + std::to_string(tvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[3]), 5);
	mLogFile->append(TAG + "transV t1: " + std::to_string(tvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[4]), 5);
	mLogFile->append(TAG + "transV t2: " + std::to_string(tvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[5]), 5);

	double *perViewErrors_data = (double*)(in_PerViewErrors.data);
	mLogFile->append("\n" + TAG + "------------- additional information -------------");
	mLogFile->append(TAG + "per preview error: " + std::to_string(perViewErrors_data[0]), 4);
	mLogFile->append(TAG + "reprojection error: " + std::to_string(mReproError4JSON) + "\n", 4);
}





// read and write correlation_camera_matrix
// currently implemented for fixed aspect ratio and dist_coeffs = 5

/**
 * @fn	void Matching::output_correlation_matrix_intO_extO(cv::Mat &_correlation_matrix, bool fix_aspect_ratio_fx_fy)
 *
 * @brief	Output correlation matrix int o extent o.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	_correlation_matrix   	The correlation matrix.
 * @param 		  	fix_aspect_ratio_fx_fy	True to fix aspect ratio effects fy.
 */

void Matching::output_correlation_matrix_intO_extO(cv::Mat &_correlation_matrix, bool fix_aspect_ratio_fx_fy) {

	std::stringstream correlation_matrix_strs;
	int number_parameters;

	if (fix_aspect_ratio_fx_fy) {
		number_parameters = 14;
		correlation_matrix_strs
			<< "r_x" << "\t" << "r_y" << "\t" << "r_z" << "\t"
			<< "t_x" << "\t" << "t_y" << "\t" << "t_z" << "\t"
			<< "fx_fy" << "\t" << "c_x" << "\t" << "c_y" << "\t"
			<< "k_1" << "\t" << "k_2" << "\t" << "p_1" << "\t" << "p_2" << "\t" << "k_3" << std::endl;
	}
	else {
		number_parameters = 15;
		correlation_matrix_strs
			<< "r_x" << "\t" << "r_y" << "\t" << "r_z" << "\t"
			<< "t_x" << "\t" << "t_y" << "\t" << "t_z" << "\t"
			<< "fx" << "\t" << "fy" << "\t" << "c_x" << "\t" << "c_y" << "\t"
			<< "k_1" << "\t" << "k_2" << "\t" << "p_1" << "\t" << "p_2" << "\t" << "k_3" << std::endl;

	}
	int counter = 0;

	// TODO check is aspect_ratio is fixed
	for (int r = 0; r < _correlation_matrix.rows; r++) {
		for (int c = 0; c < _correlation_matrix.cols; c++) {
			// get current row, depending on elements write rx ry rz tx ty tz fx/fy cx cy r1 .... 

			correlation_matrix_strs << std::setprecision(3) << std::fixed << _correlation_matrix.at<double>(c, r) << "\t";
			counter++;

			if (counter == number_parameters) {
				correlation_matrix_strs << "\n";
				counter = 0;
			}
		}
	}


	std::string line_corr_mat;

	mLogFile->append("");
	mLogFile->append(TAG + "correlations:");

	while (std::getline(correlation_matrix_strs, line_corr_mat)) {
		mLogFile->append("\t\t" + line_corr_mat);
	}

}

// output matching points of object_points_3D, image_points_2D_real, image_points_2D_synth to ...\workingDir\Matching\output.txt 

/**
 * @fn	void Matching::writer_Matches_Textfile( std::vector<cv::Point3d>& object_points_3D, std::vector<cv::Point2d>& image_points_2D_real, std::vector<cv::Point2d>& image_points_2D_synth)
 *
 * @brief	Writer matches textfile.
 *
 * @author	Mela
 * @date	25.04.2018
 *
 * @param [in,out]	object_points_3D	 	The object points 3D.
 * @param [in,out]	image_points_2D_real 	The image points 2D real.
 * @param [in,out]	image_points_2D_synth	The image points 2D synth.
 */

void Matching::writer_matches_txt_ellipsoid_bat(
	std::vector<cv::Point3d>& object_points_3D,
	std::vector<cv::Point2d>& image_points_2D_real,
	std::vector<cv::Point2d>& image_points_2D_synth) {

	// set path to ellipsoid file to false;
	mDataManager->setPath_to_ellipsoid_jar_batch("noDir");

	std::ofstream myfile3D, myfile3D_rrws, myfile2D_real, myfile2d_real_rrws, myfile2D_synth, myfile3D_bat_ellipsoid;

	myfile3D.open(mWorkingDirectory + "corresp_points_3D.txt");
	myfile2D_real.open(mWorkingDirectory + "corresp_points_2D_real.txt");
	myfile2D_synth.open(mWorkingDirectory + "corresp_points_2D_synth.txt");
	myfile3D_bat_ellipsoid.open(mWorkingDirectory + "corresp_points_3D_ellipsoid.bat");

	// check equality of objects sizes (must be the same for all vectors), otherwise return
	if (object_points_3D.size() != image_points_2D_real.size() || object_points_3D.size() != image_points_2D_synth.size() || image_points_2D_real.size() != image_points_2D_synth.size()) {
		mLogFile->append(TAG + "number of 3D & 2D points not matching. return");
		return;
	}

	// get number of elements, use object_points_3D representative
	size_t size_vectors = object_points_3D.size();

	for (uint counter = 0; counter < size_vectors; ++counter) {
		myfile3D << 1 << " " << counter << " " << object_points_3D.at(counter).x << " " << object_points_3D.at(counter).y << " " << object_points_3D.at(counter).z << endl;
		myfile3D_rrws << 1 << " " << counter << " " << object_points_3D.at(counter).x << " " << object_points_3D.at(counter).y << " " << object_points_3D.at(counter).z << endl;
		myfile2D_real << 1 << " " << counter << " " << image_points_2D_real.at(counter).x << " " << image_points_2D_real.at(counter).y << endl;
		myfile2D_synth << 1 << " " << counter << " " << image_points_2D_synth.at(counter).x << " " << image_points_2D_synth.at(counter).y << endl;
	}

	// close output streams
	myfile3D_rrws.close();
	myfile3D.close();
	myfile2D_real.close();
	myfile2D_synth.close();


	// write batch file for ellipsoid calculation
	std::string tempPath = mDataManager->getExeDirectory();
	std::string path_ellipsoid_jar = (tempPath.substr(0, tempPath.find_last_of("\\/")) + "\\libs\\3D_pointcloud_ellipsoid.jar").c_str();
	std::string text_batch_file_ellipsoid = "java -jar " + path_ellipsoid_jar + " " + mWorkingDirectory + "corresp_points_3D.txt";
	myfile3D_bat_ellipsoid << text_batch_file_ellipsoid;
	myfile3D_bat_ellipsoid.close();
	mDataManager->setPath_to_ellipsoid_jar_batch(mWorkingDirectory + "corresp_points_3D_ellipsoid.bat");
	

	// append log_file
	mLogFile->append(TAG + "Have written: corresp_points_3D.txt");
	mLogFile->append(TAG + "Have written: corresp_points_2D_real.txt");
	mLogFile->append(TAG + "Have written: corresp_points_2D_synth.txt");
	mLogFile->append(TAG + "Have written: corresp_points_3D_ellipsoid.bat");

}



// Abwandlung von Christians Matching Test
cv::Mat Matching::ransacTest_reimpl(std::vector<cv::Point2d>& _points1, std::vector<cv::Point2d>& _points2, bool refineF)
{
	// Convert keypoints into Point2f
	std::vector<cv::Point2d> points1(_points1);
	std::vector<cv::Point2d> points2(_points2);

	// Punkte die Test erfüllen
	std::vector<cv::Point2d> points1_good;
	std::vector<cv::Point2d> points2_good;

	std::vector<cv::Point2d> points1_passed;
	std::vector<cv::Point2d> points2_passed;

	std::vector<int> common_inliers;
	std::vector <int> indices_inliers_fund;
	std::vector <int> indices_inliers_homo;

	cv::Mat fundemental;

	double confidence = 0.95;
	double distance = 5.0;

	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(), 0);

	if (points1.size() > 0 && points2.size() > 0) {

		cv::Mat inliers_fund;
		cv::Mat Fund = cv::findFundamentalMat(
			cv::Mat(points1), cv::Mat(points2),  // matching points
			inliers_fund, // match status (inlier or outlier)
			cv::FM_RANSAC, // RANSAC method
			distance, // distance to epipolar line
			confidence); // confidence probability
						 // extract the surviving (inliers) matches


		std::cout << "RansacTest, Fund = " << std::endl << " " << Fund << std::endl << std::endl;

		// for all matches
		for (int r = 0; r < inliers_fund.rows; r++)
		{
			const uchar* value = inliers_fund.ptr<uchar>(r); // it is a valid match
			if (*value != 0)
				indices_inliers_fund.push_back(r);
		}

		std::cout << "RansacTest, Number of inliers Fundamental Mat, ideal camera (cxr): " << indices_inliers_fund.size() << std::endl;

		// prüfe glelichen Elemente in beiden Vektoren ab
		for (int i = 0; i < _points1.size(); i++)
		{
			if (std::find(indices_inliers_fund.begin(), indices_inliers_fund.end(), i) != indices_inliers_fund.end()) {
				points1_good.push_back(points1[i]);
				points2_good.push_back(points2[i]);
			}
		}

		std::cout << "RansacTest, Size After FundMat: " << points1_good.size() << ", 2: " << points2_good.size() << std::endl;

		if (refineF) {
			// The F matrix will be recomputed with all accepted matches
			points1.clear();
			points2.clear();

			// übergebe Punkte die ersten Test FundamentalMat bestanden haben
			points1 = (points1_good);
			points2 = (points2_good);

			// Compute 8-point F from all accepted matches
			if (points1.size() > 7 && points2.size() > 7) {
				fundemental = cv::findFundamentalMat(
					cv::Mat(points1), cv::Mat(points2), // matches
					cv::FM_8POINT); // 8-point method
			}

			if (points1.size() > 3 && points2.size() > 3)
			{
#ifdef DEBUG
				std::cout << "Num outMatches: " << outMatches.size() << ", Distances: ";
#endif
				// berechne nun homography 
				cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), cv::LMEDS, distance);

				for (unsigned i = 0; i < points1_good.size(); i++) {
					cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
					col.at<double>(0) = points1_good[i].x;
					col.at<double>(1) = points1_good[i].y;


					col = homography * col;
					col /= col.at<double>(2);
					double dist = sqrt(pow(col.at<double>(0) - points2_good[i].x, 2) +
						pow(col.at<double>(1) - points2_good[i].y, 2));
#ifdef DEBUG
					std::cout << dist << " ";
#endif

					if (dist < distance) {
						points1_passed.push_back(points1_good[i]);
						points2_passed.push_back(points2_good[i]);
					}
				}
		}
	}


#ifdef DEBUG
		std::cout << std::endl;
#endif
}

	_points1.clear();
	_points2.clear();

	_points1 = points1_passed;
	_points2 = points2_passed;

	std::cout << "RansacTest, Size of Passed points 1: " << points1_passed.size() << ", 2: " << points2_passed.size() << std::endl;
	return fundemental;
}


