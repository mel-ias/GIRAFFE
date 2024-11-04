
#include "Matching.h"


Matching::Matching() {
	mDataManager = nullptr;
	mLogFile = nullptr;
	mWorkingDirectory = "";
}

Matching::~Matching() {
}


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
void Matching::calculate_nn_synth_key___pts_image_pts(
        const std::vector<Vek2d>& in_synth_pts_float, // 2D points
        const std::vector<Vek3d>& in_synth_pts_3D_float, // 3D points
        const std::vector<Vek2d>& in_synth_keypoints_float,
        const std::vector<Vek2d>& in_real_keypoints_float,
        const cv::Mat& in_real_image,
        const cv::Mat& in_synth_image,
        std::vector<cv::Point2d>& out_matched_image_points_real,
        std::vector<cv::Point2d>& out_matched_image_points_synth,
        std::vector<cv::Point3d>& out_matched_object_points,
        const float in_neighbour_distance) {

        // Check input data
        assert(!in_synth_pts_float.empty());
        assert(!in_synth_pts_3D_float.empty());

        // Clear output vectors
        out_matched_image_points_synth.clear();
        out_matched_image_points_real.clear();
        out_matched_object_points.clear();

        // Log information
        mLogFile->append(TAG + "count synth_keypoints_float: " + std::to_string(in_synth_keypoints_float.size()));
        mLogFile->append(TAG + "count real_keypoints_float: " + std::to_string(in_real_keypoints_float.size()));
        mLogFile->append(TAG + "neighbour distance threshold (in Px): " + std::to_string(in_neighbour_distance));

        // Create cv::Mat for the 2D points
        cv::Mat in_synth_pts_mat(in_synth_pts_float.size(), 2, CV_32F);
        for (size_t i = 0; i < in_synth_pts_float.size(); ++i) {
            in_synth_pts_mat.at<float>(i, 0) = static_cast<float>(in_synth_pts_float[i].x()); // x-coordinate
            in_synth_pts_mat.at<float>(i, 1) = static_cast<float>(in_synth_pts_float[i].y()); // y-coordinate
        }

        // Prepare for KNN search
        cv::flann::Index kdtree(in_synth_pts_mat, cv::flann::KDTreeIndexParams());

        // Initialize canvas for output
        cv::Mat draw_canvas_synth = in_synth_image.clone();
        cv::Mat draw_canvas_real = in_real_image.clone();

        // Step through synth keypoints and apply KNN search
        uint counter = 0;
        for (const auto& p : in_synth_keypoints_float) {
            // Convert 2D keypoint to a format suitable for knnSearch (a row matrix)
            cv::Mat query_point(1, 2, CV_32F);
            query_point.at<float>(0, 0) = static_cast<float>(p.x());
            query_point.at<float>(0, 1) = static_cast<float>(p.y());

            // Perform KNN search
            std::vector<int> indices(1); // For storing the index of the nearest neighbor
            std::vector<float> dists(1); // For storing the distance to the nearest neighbor
            kdtree.knnSearch(query_point, indices, dists, 1);

            // Ensure that the nearest neighbor distance is valid
            if (dists.empty() || dists[0] > in_neighbour_distance * in_neighbour_distance) {
                counter++; // Increment counter in each case
                continue;
            }

            // Push back corresponding synth 2D, real 2D (image points) and synth 3D (object points)
            out_matched_image_points_synth.push_back(cv::Point2d(in_synth_pts_float[indices[0]].x(), in_synth_pts_float[indices[0]].y()));
            out_matched_image_points_real.push_back(cv::Point2d(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()));
            out_matched_object_points.push_back(cv::Point3d(in_synth_pts_3D_float[indices[0]].x(), in_synth_pts_3D_float[indices[0]].y(), in_synth_pts_3D_float[indices[0]].z()));

            // Output
            cv::circle(draw_canvas_synth, cv::Point(in_synth_pts_float[indices[0]].x(), in_synth_pts_float[indices[0]].y()), 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(draw_canvas_synth, cv::Point(p.x(), p.y()), 1, cv::Scalar(0, 255, 0), 3);
            cv::circle(draw_canvas_real, cv::Point(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()), 1, cv::Scalar(0, 255, 0), 3);

            std::string counterTxt = std::to_string(counter) + "," +
                std::to_string(in_synth_pts_3D_float[indices[0]].x()) + "," +
                std::to_string(in_synth_pts_3D_float[indices[0]].y()) + "," +
                std::to_string(in_synth_pts_3D_float[indices[0]].z());

            cv::putText(draw_canvas_real, std::to_string(counter), cv::Point(in_real_keypoints_float[counter].x(), in_real_keypoints_float[counter].y()), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
            cv::putText(draw_canvas_synth, std::to_string(counter), cv::Point(in_synth_pts_float[indices[0]].x(), in_synth_pts_float[indices[0]].y()), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));

            counter++; // Increment counter in each case
        }

        // Log output
        mLogFile->append(TAG + "size matched_objectPoints/ matched_imagePoints_synth/ matched_imagePoints_real: " +
            std::to_string(out_matched_object_points.size()) + "/" +
            std::to_string(out_matched_image_points_synth.size()) + "/" +
            std::to_string(out_matched_image_points_real.size()));
        mLogFile->append(TAG + "finish matching");

        // Save output images
        cv::imwrite(mWorkingDirectory + "corresp_points_synth.png", draw_canvas_synth);
        cv::imwrite(mWorkingDirectory + "corresp_points_real.png", draw_canvas_real);
    }





/**
 * @fn	void Matching::loadMatches( std::string in_path_FMatrixOutput, cv::Mat&amp; in_real_image, cv::Mat&amp; in_synth_image, std::vector&lt;cv::Point2d&gt;&amp; in_wl_pts_2D, std::vector&lt;Vek2d&gt;&amp; in_synth_pts_2D, std::vector&lt;Vek3d&gt;&amp; in_synth_pts_3D, std::vector&lt;cv::Point3d&gt;&amp; out_matched_object_points, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_real, std::vector&lt;cv::Point2d&gt;&amp; out_matched_image_points_synth)
 *
 * @summary	public function read f-inlier matches from SiftGPU (included in Visual SFM console
 * 			application)
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

		
		// create an input string stream
		std::istringstream stm(line);
		int id;
		double img1_x, img1_y, img2_x, img2_y; //coordinates of feature point inliers; img1 = real image; img2 = synth image
		stm >> id >> img1_x >> img1_y >> img2_x >> img2_y;

		// push back inlieres separately
		real_matched_pts.push_back(cv::Point2d(img1_x, img1_y));
		synth_matched_pts.push_back(cv::Point2d(img2_x, img2_y));

		counter++;
		
	}

	mLogFile->append(TAG + "count fundamental inlier matches (real_matched_pts/synth_matched_pts):" + std::to_string(real_matched_pts.size()) + "/" + std::to_string(synth_matched_pts.size()));

	// check for outlieres
	ransac_test(real_matched_pts, synth_matched_pts);


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

	// output to ascii point cloud file (matched_object_points, matched_image_points_real, matched_imagePoints_synth)
	write_corresponding_points_to_file(out_matched_object_points, out_matched_image_points_real, out_matched_image_points_synth);

	// draw matches
	write_visualization_matches(in_real_image, in_synth_image, real_matched_pts_Vek2d, synth_matched_pts_Vek2d, "matches_2D");

	// update 02.07.22 because function above do not print inlier matches alone!
	std::vector<Vek2d> out_matched_image_points_real_Vek2d, out_matched_image_points_synth_Vek2d;
	for (cv::Point p : out_matched_image_points_real)
		out_matched_image_points_real_Vek2d.push_back(Vek2d(p.x, p.y));

	for (cv::Point p : out_matched_image_points_synth)
		out_matched_image_points_synth_Vek2d.push_back(Vek2d(p.x, p.y));
	write_visualization_matches(in_real_image, in_synth_image, out_matched_image_points_real_Vek2d, out_matched_image_points_synth_Vek2d, "matches_with_3D_val");

}



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
 * @param[in] in_pix_size Pixel size (used for scaling).
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
 float Matching::space_resection(std::vector<cv::Point3d>& in_matched_object_points,
	 std::vector<cv::Point2d>& in_matched_image_points_real,
	 cv::Mat& true_image,
	 double in_pix_size,
	 cv::Mat& camera_matrix,
	 cv::Mat& dist_coeffs,
	 cv::Mat& rvec,
	 cv::Mat& tvec,
	 cv::Mat& stdDev_In,
	 cv::Mat& stdDev_Ext,
	 Flags_resec in_flag,
	 bool fisheye)
 {
	 
	 // Early return if both extrinsic and intrinsic parameters are fixed
	 if (in_flag == FIXED_EO_IO) {
		 mLogFile->append(TAG + " Both EO and IO fixed.");
		 return -1;
	 }
	 
	 // 1. Validate input parameters and data
	 if (camera_matrix.total() != 9 || dist_coeffs.empty() ||
		 in_matched_object_points.size() < 4 || in_matched_image_points_real.size() < 4) {
		 mLogFile->append(TAG + ", invalid inputs.");
		 return -1;
	 }

	 // 2. Define parameters for RANSAC
	 constexpr int iterationsCount = 10000;
	 const float solvePnPRansac_reproErr = fisheye ?
		 mDataManager->get_filter_matches_ransac_fisheye() :
		 mDataManager->get_filter_matches_ransac_pinhole();

	 std::vector<int> inliers;
	 float repro_error = -1.0f;

	 // Optional: Set a deterministic random seed to make RANSAC reproducible
	 cv::theRNG().state = 42;

	 // Check if rvec is a 3x3 matrix
	 if (rvec.rows == 3 && rvec.cols == 3) {
		 cv::Rodrigues(rvec, rvec);
	 }

	 // 3. Run solvePnPRansac for initial pose estimation and outlier filtering
	 if (!cv::solvePnPRansac(in_matched_object_points, in_matched_image_points_real,
		 camera_matrix, dist_coeffs, rvec, tvec, true,
		 iterationsCount, solvePnPRansac_reproErr, 0.9999,
		 inliers, cv::SOLVEPNP_ITERATIVE)) {
		 mLogFile->append(TAG + ", solvePnPRansac failed.");
		 return -1;
	 }

	 // 4. Collect inliers
	 std::vector<cv::Point3d> object_points_ransac;
	 std::vector<cv::Point2d> image_points_real_ransac;
	 for (int i : inliers) {
		 image_points_real_ransac.push_back(in_matched_image_points_real[i]);
		 object_points_ransac.push_back(in_matched_object_points[i]);
	 }

	 // 5. Optimize extrinsics or both intrinsic and extrinsic parameters
	 cv::TermCriteria termCrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, DBL_EPSILON);
	 /*if (in_flag == CALC_EO) {
		 mLogFile->append(TAG + " Optimizing only EO via solvePnPRefineLM.");
		 cv::solvePnPRefineLM(object_points_ransac, image_points_real_ransac,
			 camera_matrix, dist_coeffs, rvec, tvec, termCrit);
	 }*/
	 if (in_flag ==!CALC_EO) {
		 mLogFile->append(TAG + " Optimizing both EO and IO.");
		 std::vector<std::vector<cv::Point3d>> obj_pts_vector = { object_points_ransac };
		 std::vector<std::vector<cv::Point2d>> img_pts_vector = { image_points_real_ransac };

		 if (!fisheye) {
			 int flags = cv::CALIB_USE_INTRINSIC_GUESS;
			 repro_error = cv::calibrateCamera(obj_pts_vector, img_pts_vector,
				 true_image.size(), camera_matrix, dist_coeffs,
				 rvec, tvec, stdDev_In, stdDev_Ext, cv::Mat(), flags, termCrit);
		 }
		 else {
			 std::vector<cv::Mat> rvecs, tvecs;
			 int flags = cv::fisheye::CALIB_USE_INTRINSIC_GUESS | cv::fisheye::CALIB_FIX_SKEW |
				 cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_K4 |
				 cv::fisheye::CALIB_FIX_K3 | cv::fisheye::CALIB_FIX_K2 | cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
			 repro_error = cv::fisheye::calibrate(obj_pts_vector, img_pts_vector,
				 true_image.size(), camera_matrix, dist_coeffs,
				 rvecs, tvecs, flags, termCrit);
		 }
	 }

	 // 6. Log the optimized translation and rotation vectors
	 std::ostringstream t_vec_str, r_vec_str;
	 t_vec_str << tvec;
	 r_vec_str << rvec;
	 mLogFile->append(TAG + " Optimized tvec / rvec: " + t_vec_str.str() + " / " + r_vec_str.str());

	 // 7. Compute reprojected points and standard deviations
	 std::vector<cv::Point2d> reprojected_points;
	 cv::Mat jacobian;
	 if (fisheye) {
		 cv::fisheye::projectPoints(object_points_ransac, reprojected_points, rvec, tvec, camera_matrix, dist_coeffs, 0, jacobian);
	 }
	 else {
		 cv::projectPoints(object_points_ransac, rvec, tvec, camera_matrix, dist_coeffs, reprojected_points, jacobian);
	 }

	 // 8. Calculate standard deviations if not fisheye
	 if (!fisheye && !jacobian.empty()) {
		 cv::Mat sigma_extr = cv::Mat(jacobian.t() * jacobian, cv::Rect(0, 0, 6, 6)).inv();
		 cv::Mat sigma_intr = cv::Mat(jacobian.t() * jacobian, cv::Rect(6, 6, 9, 9)).inv();
		 cv::sqrt(sigma_extr.diag(), stdDev_Ext);
		 cv::sqrt(sigma_intr.diag(), stdDev_In);
	 }

	 // 9. Compute reprojection error
	 repro_error = 0.0f;
	 for (size_t i = 0; i < image_points_real_ransac.size(); ++i) {
		 repro_error += cv::norm(image_points_real_ransac[i] - reprojected_points[i]);
	 }
	 repro_error /= static_cast<float>(object_points_ransac.size());

	 // 10. Ensure rvec and tvec are single-channel
	 if (rvec.channels() == 3 || tvec.channels() == 3) {
		 rvec = rvec.reshape(1);
		 tvec = tvec.reshape(1);
	 }

	 return repro_error;
 }





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
cv::Mat Matching::ransac_test(std::vector<cv::Point2d>& _points1, std::vector<cv::Point2d>& _points2, double confidence, double distance, bool refineF)
{
	// Copy input points into local vectors for processing
	std::vector<cv::Point2d> points1(_points1);
	std::vector<cv::Point2d> points2(_points2);

	// Vectors to hold inlier points that pass the RANSAC test
	std::vector<cv::Point2d> points1_good;
	std::vector<cv::Point2d> points2_good;

	// Vectors for points that pass both fundamental matrix and homography refinement
	std::vector<cv::Point2d> points1_passed;
	std::vector<cv::Point2d> points2_passed;

	// Inlier indices for fundamental matrix and homography estimations
	std::vector<int> indices_inliers_fund;
	std::vector<int> indices_inliers_homo;

	// Fundamental matrix placeholder
	cv::Mat fundamental;

	// Vector to store the inliers' status from the RANSAC algorithm
	std::vector<uchar> inliers(points1.size(), 0);

	// Ensure there are matching points to process
	if (points1.size() > 0 && points2.size() > 0) {

		// Inlier mask for the fundamental matrix estimation
		cv::Mat inliers_fund;

		// Estimate the fundamental matrix using RANSAC
		cv::Mat Fund = cv::findFundamentalMat(
			cv::Mat(points1), cv::Mat(points2),  // Input point sets
			inliers_fund,                        // Inlier mask
			cv::FM_RANSAC,                       // Use RANSAC for robust estimation
			distance,                            // Distance threshold for inliers
			confidence);                         // Confidence level for RANSAC

		// Debug: Output the estimated fundamental matrix
		// std::cout << "RansacTest, Fundamental Matrix = " << std::endl << Fund << std::endl;

		// Collect the inlier indices based on the RANSAC result
		for (int r = 0; r < inliers_fund.rows; r++) {
			const uchar* value = inliers_fund.ptr<uchar>(r);
			if (*value != 0) // If the point is an inlier
				indices_inliers_fund.push_back(r);
		}

		// Debug
		// std::cout << "RansacTest, Number of inliers (Fundamental Matrix): " << indices_inliers_fund.size() << std::endl;

		// Filter the points that passed the RANSAC test by matching inliers
		for (int i = 0; i < _points1.size(); i++) {
			if (std::find(indices_inliers_fund.begin(), indices_inliers_fund.end(), i) != indices_inliers_fund.end()) {
				points1_good.push_back(points1[i]);
				points2_good.push_back(points2[i]);
			}
		}

		// Update input vectors with the inlier points
		_points1.clear();
		_points2.clear();
		_points1 = points1_good;
		_points2 = points2_good;

		// Debug
		// std::cout << "RansacTest, Size After Fundamental Matrix Filtering: " << _points1.size() << ", " << _points2.size() << std::endl;

		// Optionally refine the fundamental matrix using inlier points
		if (refineF) {
			// Clear local vectors to prepare for final computation
			points1.clear();
			points2.clear();

			// Use points that passed the initial fundamental matrix test
			points1 = points1_good;
			points2 = points2_good;

			// Recompute the fundamental matrix using the 8-point method
			if (points1.size() > 7 && points2.size() > 7) {
				fundamental = cv::findFundamentalMat(cv::Mat(points1), cv::Mat(points2), cv::FM_8POINT);
			}

			// Further refine matches by estimating the homography matrix
			if (points1.size() > 3 && points2.size() > 3) {
				// Compute homography using the Least-Median of Squares method
				cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), cv::LMEDS, distance);

				// Evaluate each inlier based on the homography transformation
				for (unsigned i = 0; i < points1_good.size(); i++) {
					// Convert the point to homogeneous coordinates
					cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
					col.at<double>(0) = points1_good[i].x;
					col.at<double>(1) = points1_good[i].y;

					// Apply the homography transformation
					col = homography * col;
					col /= col.at<double>(2);

					// Compute the Euclidean distance between transformed points
					double dist = sqrt(pow(col.at<double>(0) - points2_good[i].x, 2) + pow(col.at<double>(1) - points2_good[i].y, 2));

					// Print the distance for debugging purposes
					// std::cout << dist << " ";

					// Filter points based on distance threshold
					if (dist < distance) {
						points1_passed.push_back(points1_good[i]);
						points2_passed.push_back(points2_good[i]);
					}
				}
			}

			// Update the input vectors with points that passed the homography test
			_points1.clear();
			_points2.clear();
			_points1 = points1_passed;
			_points2 = points2_passed;
		}
	}

	// Output the final number of points that passed both tests
	char buf_pt1[24], buf_pt2[24];
	_ultoa(_points1.size(), buf_pt1, 10);
	_ultoa(_points2.size(), buf_pt2, 10);
	mLogFile->append(TAG + "RansacTest, Final Number of Passed Points " + std::string(buf_pt1) + "," + std::string(buf_pt2) + "\n");

	// Return the final fundamental matrix
	return fundamental;
}





void Matching::image_points_3D_referencing(std::vector<cv::Point2d>& input_image_points, std::vector<Vek3d>& synth_pts_3D, cv::Mat& in_image_4_color, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec_cc_orig_copy, cv::Mat& tvec_cc_orig_copy, double shift_x, double shift_y, double shift_z, bool export_pcl, std::string file_name_image_points) {

	std::stringstream log_statistics;

	// undistort water line points and push back into image coordinate system
	std::vector<cv::Point2d> img_pts_2D_undistort_normalized_coordinates;
	std::vector<cv::Point2d> img_pts_2D_undistort_image_coordinates;

	Model model = Model(mLogFile);
	std::vector<cv::Vec3b> point_cloud_color;
	std::vector<cv::Point2d> image_coordinates_color;
	double distance_threshold_img_to_proj_img = 2.0; // pixels

	if (input_image_points.size() != 0) {
		// undistort 2D image points of water line
		cv::undistortPoints(input_image_points, img_pts_2D_undistort_normalized_coordinates, camera_matrix, dist_coeffs);

		// conversion to image coordinates
		for (cv::Point2d p : img_pts_2D_undistort_normalized_coordinates) {
			img_pts_2D_undistort_image_coordinates.push_back(cv::Point2d(
				camera_matrix.at<double>(0, 0) * p.x + camera_matrix.at<double>(0, 2),
				camera_matrix.at<double>(1, 1) * p.y + camera_matrix.at<double>(1, 2)));
		}
	}
	else {
		mLogFile->append(TAG + "no image points to reference provided, will only color point cloud");
	}

	// convert synth_pts_3D to cv::Point3d
	std::vector<cv::Point3d> synth_pts_3D_cv;
	for (Vek3d vec : synth_pts_3D) {
		synth_pts_3D_cv.push_back(cv::Point3d(vec.x(), vec.y(), vec.z()));
	}

	// Get color and project 2D waterline points to object space using OpenCV's projectPoints
	Model::ReferencedPoints referenced_points = model.getColorFor(synth_pts_3D_cv, in_image_4_color, point_cloud_color, image_coordinates_color, 1.0, camera_matrix, dist_coeffs, rvec_cc_orig_copy, tvec_cc_orig_copy, input_image_points);
	

	// Print point cloud if required
	if (export_pcl) {
		mLogFile->append(TAG + "---- export 3D point cloud ----");
		model.export_point_cloud_recolored(mWorkingDirectory, mDataManager->get_name_working_directory(), synth_pts_3D_cv, point_cloud_color, image_coordinates_color, shift_x, shift_y, shift_z);
	}

	// If projection succeeded, log statistics and export results
	if (!referenced_points.corresponding_3D_image_pts_from_point_cloud.empty()) {
		
		// outlier removal 
		Matching::FilteredData filtered_data = filterPointsByDistance(input_image_points, referenced_points.corresponding_2D_image_pts_from_point_cloud, referenced_points.corresponding_3D_image_pts_from_point_cloud, distance_threshold_img_to_proj_img);
		
		cv::Mat copy_masterimage = in_image_4_color.clone();
		// Draw original image points
		for (cv::Point2d p : input_image_points) {
			cv::circle(copy_masterimage, p, 5, cv::Scalar(255, 255, 0), -1);
		}
		// Draw projected image points
		for (cv::Point2d p : filtered_data.image_data_projected) {
			cv::circle(copy_masterimage, p, 4, cv::Scalar(255, 0, 255), -1);
		}
		cv::imwrite(mWorkingDirectory + "projected_original_image_points.png", copy_masterimage);

		// Write projected points and IDs to files
		std::ofstream myfile(mWorkingDirectory + file_name_image_points + "_projected.txt");	
		for (int i = 0; i < filtered_data.image_data_3D.size(); i++) {
			myfile << std::fixed << std::setprecision(4)
				<< filtered_data.image_data_original_idx[i] << ","
				<< filtered_data.image_data_3D[i].x + shift_x << ","
				<< filtered_data.image_data_3D[i].y + shift_y << ","
				<< filtered_data.image_data_3D[i].z + shift_z << "\n";
		}
		myfile.close();
		mLogFile->append(TAG + "count referenced image points: " + std::to_string(filtered_data.image_data_3D.size()));


		
	}
	else {
		mLogFile->append(TAG + "no water level detected");
	}
}









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
void Matching::write_visualization_matches(cv::Mat& in_real_image, cv::Mat& in_synth_image, std::vector<Vek2d>& in_real_matches_draw, std::vector<Vek2d>& in_synth_matches_draw, std::string fileName) {

	#ifdef max
	#undef max
	#endif
	// Create a new image to combine both real and synthetic images side by side
	cv::Mat matchesImage = cv::Mat(
		std::max(in_synth_image.rows, in_real_image.rows),    // Choose the larger height between the two images
		in_synth_image.cols + in_real_image.cols,             // Sum of the widths of both images
		in_synth_image.type(),                                // Set the image type (same as the synthetic image)
		cv::Scalar(0, 0, 0));                                 // Initialize with black background

	mLogFile->append(TAG + "Visualizing matches between real and synthetic images");

	// Copy the real image to the left side of the combined image
	for (int i = 0; i < matchesImage.rows; ++i) {
		for (int j = 0; j < matchesImage.cols; ++j) {
			if (j < in_real_image.cols && i < in_real_image.rows) {
				matchesImage.at<cv::Vec3b>(i, j) = in_real_image.at<cv::Vec3b>(i, j);
			}
		}
	}

	// Copy the synthetic image to the right side of the combined image
	for (int i = 0; i < matchesImage.rows; ++i) {
		for (int j = in_real_image.cols; j < matchesImage.cols; ++j) {
			if (j - in_real_image.cols < in_synth_image.cols && i < in_synth_image.rows) {
				matchesImage.at<cv::Vec3b>(i, j) = in_synth_image.at<cv::Vec3b>(i, j - in_real_image.cols);
			}
		}
	}

	// Draw lines connecting the corresponding keypoints between the real and synthetic images
	for (int i = 0; i < static_cast<int>(in_real_matches_draw.size()); ++i) {
		// Set color for the keypoint connection (red for matches)
		cv::Scalar matchColor(0, 0, 255);

		// Extract keypoints from the real and synthetic images
		cv::Point2d point_real = cv::Point2d(in_real_matches_draw[i].x(), in_real_matches_draw[i].y());
		cv::Point2d point_synth = cv::Point2d(in_synth_matches_draw[i].x(), in_synth_matches_draw[i].y());
		point_synth.x += in_real_image.cols; // Shift synthetic points by real image width for correct positioning

		// Draw circles around the keypoints
		circle(matchesImage, point_real, 3, matchColor, 1);  // Real image keypoints in red
		circle(matchesImage, point_synth, 3, matchColor, 1); // Synthetic image keypoints in red

		// Draw lines connecting the keypoints between the real and synthetic images
		cv::line(matchesImage, point_real, point_synth, matchColor, 2, 8, 0);
	}

	// Save the final image with the drawn matches
	cv::imwrite(mWorkingDirectory + fileName + ".jpg", matchesImage);
}









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
void Matching::write_camera_calibration_statistics(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& in_StdDev_IntO, cv::Mat& in_StdDev_ExtO, cv::Mat& in_PerViewErrors, double in_pix_size) {
	// Add a blank line for formatting in the log file and indicate the start of camera calibration results
	mLogFile->append("");
	mLogFile->append(TAG + "---- camera calibration results ----");

	// Access the data of the intrinsic camera matrix and standard deviations
	// ocv_docu: A(0, 0) = param[0]; A(1, 1) = param[1]; A(0, 2) = param[2]; A(1, 2) = param[3]; std::copy(param + 4, param + 4 + 14, k);
	double* camera_matrix_data = (double*)(in_camera_matrix.data);
	double* stdDev_In_data = (double*)(in_StdDev_IntO.data);

	// Log the intrinsic parameters of the camera (focal lengths, optical center) in both pixels and mm
	mLogFile->append("\n" + TAG + "------------- intrinsics -------------");
	mLogFile->append(TAG + "fx: " + std::to_string(camera_matrix_data[0]) + " [px]/ " + std::to_string(camera_matrix_data[0] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[0]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[0] * in_pix_size), 5);
	mLogFile->append(TAG + "fy: " + std::to_string(camera_matrix_data[4]) + " [px]/ " + std::to_string(camera_matrix_data[4] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[1]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[1] * in_pix_size), 5);
	mLogFile->append(TAG + "cx: " + std::to_string(camera_matrix_data[2]) + " [px]/ " + std::to_string(camera_matrix_data[2] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[2]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[2] * in_pix_size), 5);
	mLogFile->append(TAG + "cy: " + std::to_string(camera_matrix_data[5]) + " [px]/ " + std::to_string(camera_matrix_data[5] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[3]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[3] * in_pix_size), 5);

	// Access the distortion coefficients and log them along with their standard deviations
	double* dist_coeffs_data = (double*)(in_dist_coeffs.data);
	mLogFile->append("\n" + TAG + "------------- distortion coefficients -------------");
	mLogFile->append(TAG + "k1: " + std::to_string(dist_coeffs_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[4]), 8);
	mLogFile->append(TAG + "k2: " + std::to_string(dist_coeffs_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[5]), 8);
	mLogFile->append(TAG + "p1: " + std::to_string(dist_coeffs_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[6]), 8);
	mLogFile->append(TAG + "p2: " + std::to_string(dist_coeffs_data[3]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[7]), 8);
	mLogFile->append(TAG + "k3: " + std::to_string(dist_coeffs_data[4]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[8]), 8);

	// Access the rotation and translation vectors along with their standard deviations
	double* rvec_cc_1ch_data = (double*)(in_rvec.data);
	double* tvec_cc_1ch_data = (double*)(in_tvec.data);
	double* stdDev_Ext_data = (double*)(in_StdDev_ExtO.data);
	mLogFile->append("\n" + TAG + "------------- extrinsics -------------");
	mLogFile->append(TAG + "rotV r0: " + std::to_string(rvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[0]), 5);
	mLogFile->append(TAG + "rotV r1: " + std::to_string(rvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[1]), 5);
	mLogFile->append(TAG + "rotV r2: " + std::to_string(rvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[2]), 5);
	mLogFile->append(TAG + "transV t0: " + std::to_string(tvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[3]), 5);
	mLogFile->append(TAG + "transV t1: " + std::to_string(tvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[4]), 5);
	mLogFile->append(TAG + "transV t2: " + std::to_string(tvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[5]), 5);

	// Access the per-view errors and log the reprojection error
	double* perViewErrors_data = (double*)(in_PerViewErrors.data);
	mLogFile->append("\n" + TAG + "------------- additional information -------------");
	mLogFile->append(TAG + "per-view error: " + std::to_string(perViewErrors_data[0]), 4);
}



/**
 * @brief Writes log statistics to a file and prints them to the console.
 *
 * This method extracts log data from a stringstream and writes it to a log file, while also printing each line
 * to the console for immediate feedback. The log file is saved in the working directory under the name
 * "image_points_projected.txt".
 *
 * @param log_statistics A stringstream containing the statistics to be logged.
 */
void Matching::write_and_print_log_statistics(std::stringstream& log_statistics) const {
	std::string line;
	std::ofstream stats_file;

	// Open the output file in the working directory to store log statistics
	stats_file.open(mWorkingDirectory + "image_points_projected_stats.txt");

	// Iterate through each line of the log statistics stringstream
	while (std::getline(log_statistics, line)) {
		// Print each line to the console
		std::cout << line << std::endl;
	}

	// Close the file stream after writing
	stats_file.close();
}



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
void Matching::write_corresponding_points_to_file(
	std::vector<cv::Point3d>& object_points_3D,
	std::vector<cv::Point2d>& image_points_2D_real,
	std::vector<cv::Point2d>& image_points_2D_synth)
{
	// Create output file streams for 3D, 2D real, and 2D synthetic points
	std::ofstream myfile3D, myfile3D_rrws, myfile2D_real, myfile2D_synth;

	// Open files in the working directory for writing
	myfile3D.open(mWorkingDirectory + "corresp_points_3D.txt");
	myfile2D_real.open(mWorkingDirectory + "corresp_points_2D_real.txt");
	myfile2D_synth.open(mWorkingDirectory + "corresp_points_2D_synth.txt");

	// Check if the size of all input vectors is equal; if not, log and return
	if (object_points_3D.size() != image_points_2D_real.size() ||
		object_points_3D.size() != image_points_2D_synth.size() ||
		image_points_2D_real.size() != image_points_2D_synth.size())
	{
		mLogFile->append(TAG + "number of 3D & 2D points not matching. return");
		return;
	}

	// Get the number of elements to process
	size_t size_vectors = object_points_3D.size();

	// Iterate over the points and write each to its corresponding file
	for (uint counter = 0; counter < size_vectors; ++counter) {
		// Write the 3D points to the file (1 is a prefix used in the output format)
		myfile3D << 1 << " " << counter << " "
			<< object_points_3D.at(counter).x << " "
			<< object_points_3D.at(counter).y << " "
			<< object_points_3D.at(counter).z << std::endl;

		// Write the real 2D points to the file
		myfile2D_real << 1 << " " << counter << " "
			<< image_points_2D_real.at(counter).x << " "
			<< image_points_2D_real.at(counter).y << std::endl;

		// Write the synthetic 2D points to the file
		myfile2D_synth << 1 << " " << counter << " "
			<< image_points_2D_synth.at(counter).x << " "
			<< image_points_2D_synth.at(counter).y << std::endl;
	}

	// Close the output streams to finalize the files
	myfile3D.close();
	myfile2D_real.close();
	myfile2D_synth.close();

	// Log the successful creation of the output files
	mLogFile->append(TAG + "Have written: corresp_points_3D.txt");
	mLogFile->append(TAG + "Have written: corresp_points_2D_real.txt");
	mLogFile->append(TAG + "Have written: corresp_points_2D_synth.txt");

}


