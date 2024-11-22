#include "Matching.h"


Matching::Matching() {
	_data_manager = nullptr;   // Initialize data manager pointer to null
	_logfile = nullptr;        // Initialize log file pointer to null
	_working_dir_matching = ""; // Initialize working directory path as empty
}


Matching::~Matching() {
	// Currently, no dynamic memory or resources to release.
}


void Matching::init(DataManager* _dataManager) {

	// Initialize log file for tracking Matching process steps
	_logfile = _dataManager->get_logfile();
	_logfile->append("");
	_logfile->append(TAG + "---- initialisation matching ----");

	// Store reference to the DataManager for future data access
	_data_manager = _dataManager;

	// Define the path for the "Matching" working directory
	_working_dir_matching = _data_manager->get_path_working_directory() / "Matching";

	// Check if the directory exists; if not, create it
	if (!fs::exists(_working_dir_matching)) {
		fs::create_directory(_working_dir_matching);
	}
}


void Matching::calculate_nn_synth_key___pts_image_pts(
    const std::vector<cv::Point2d>& in_synth_pts_float, // 2D points
    const std::vector<cv::Point3d>& in_synth_pts_3D_float, // 3D points
    const std::vector<cv::Point2d>& in_synth_keypoints_float,
    const std::vector<cv::Point2d>& in_real_keypoints_float,
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
    _logfile->append(TAG + "count synth_keypoints_float: " + std::to_string(in_synth_keypoints_float.size()));
    _logfile->append(TAG + "count real_keypoints_float: " + std::to_string(in_real_keypoints_float.size()));
    _logfile->append(TAG + "neighbour distance threshold (in Px): " + std::to_string(in_neighbour_distance));

    // Create cv::Mat for the 2D points
    cv::Mat in_synth_pts_mat(in_synth_pts_float.size(), 2, CV_32F);
    for (size_t i = 0; i < in_synth_pts_float.size(); ++i) {
        in_synth_pts_mat.at<float>(i, 0) = static_cast<float>(in_synth_pts_float[i].x); // x-coordinate
        in_synth_pts_mat.at<float>(i, 1) = static_cast<float>(in_synth_pts_float[i].y); // y-coordinate
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
        query_point.at<float>(0, 0) = static_cast<float>(p.x);
        query_point.at<float>(0, 1) = static_cast<float>(p.y);

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
        out_matched_image_points_synth.push_back(cv::Point2d(in_synth_pts_float[indices[0]].x, in_synth_pts_float[indices[0]].y));
        out_matched_image_points_real.push_back(cv::Point2d(in_real_keypoints_float[counter].x, in_real_keypoints_float[counter].y));
        out_matched_object_points.push_back(cv::Point3d(in_synth_pts_3D_float[indices[0]].x, in_synth_pts_3D_float[indices[0]].y, in_synth_pts_3D_float[indices[0]].z));

        // Output
        cv::circle(draw_canvas_synth, cv::Point(in_synth_pts_float[indices[0]].x, in_synth_pts_float[indices[0]].y), 5, cv::Scalar(0, 0, 255), -1);
        cv::circle(draw_canvas_synth, cv::Point(p.x, p.y), 1, cv::Scalar(0, 255, 0), 3);
        cv::circle(draw_canvas_real, cv::Point(in_real_keypoints_float[counter].x, in_real_keypoints_float[counter].y), 1, cv::Scalar(0, 255, 0), 3);

        std::string counterTxt = std::to_string(counter) + "," +
            std::to_string(in_synth_pts_3D_float[indices[0]].x) + "," +
            std::to_string(in_synth_pts_3D_float[indices[0]].y) + "," +
            std::to_string(in_synth_pts_3D_float[indices[0]].z);

        cv::putText(draw_canvas_real, std::to_string(counter), cv::Point(in_real_keypoints_float[counter].x, in_real_keypoints_float[counter].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
        cv::putText(draw_canvas_synth, std::to_string(counter), cv::Point(in_synth_pts_float[indices[0]].x, in_synth_pts_float[indices[0]].y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));

        counter++; // Increment counter in each case
    }

    // Log output
    _logfile->append(TAG + "size matched_objectPoints/ matched_imagePoints_synth/ matched_imagePoints_real: " +
        std::to_string(out_matched_object_points.size()) + "/" +
        std::to_string(out_matched_image_points_synth.size()) + "/" +
        std::to_string(out_matched_image_points_real.size()));
    _logfile->append(TAG + "finish matching");

    // Save output images
	cv::imwrite(fs::path(_working_dir_matching / "corresp_points_synth.png").string(), draw_canvas_synth);
	cv::imwrite(fs::path(_working_dir_matching / "corresp_points_real.png").string(), draw_canvas_real);
}


void Matching::loadMatches(
	std::string in_path_matching_output,
	cv::Mat& in_real_image,
	cv::Mat& in_synth_image,
	std::vector<cv::Point2d>& in_synth_pts_2D,
	std::vector<cv::Point3d>& in_synth_pts_3D,
	std::vector<cv::Point3d>& out_matched_object_points,
	std::vector<cv::Point2d>& out_matched_image_points_real,
	std::vector<cv::Point2d>& out_matched_image_points_synth,
	float neighbour_distance_allowed_pointcloud) {

	_logfile->append("");
	_logfile->append(TAG + "read matching results");
	_logfile->append(TAG + "real_image: " + std::to_string(in_real_image.size().width) + "x" + std::to_string(in_real_image.size().height) + "," + std::to_string(in_real_image.type()));
	_logfile->append(TAG + "synth_image: " + std::to_string(in_synth_image.size().width) + "x" + std::to_string(in_synth_image.size().height) + "," + std::to_string(in_synth_image.type()));

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

	_logfile->append(TAG + "count fundamental inlier matches (real_matched_pts/synth_matched_pts):" + std::to_string(real_matched_pts.size()) + "/" + std::to_string(synth_matched_pts.size()));

	// check for outliers
	ransac_test(real_matched_pts, synth_matched_pts);

	// data conversion for image matching
	// receive 3D coordinates for all image points 
	// vectors for matching results in declaration
	calculate_nn_synth_key___pts_image_pts(
		in_synth_pts_2D, in_synth_pts_3D, synth_matched_pts, real_matched_pts,
		in_real_image, in_synth_image,
		out_matched_image_points_real, out_matched_image_points_synth, out_matched_object_points,
		neighbour_distance_allowed_pointcloud);

	// check sizes of matched point vectors (2D synth, real, 3D synth_object), must be equal otherwise return
	if (out_matched_object_points.size() == 0 || out_matched_image_points_real.size() == 0 || out_matched_image_points_real.size() != out_matched_object_points.size()) {
		_logfile->append("Problem with determination of corresponding points between matched synthetic and matched real image and 3D point cloud data.");
		return;
	}

	// output to ascii point cloud file (matched_object_points, matched_image_points_real, matched_imagePoints_synth)
	write_corresponding_points_to_file(out_matched_object_points, out_matched_image_points_real, out_matched_image_points_synth);

	// draw matches
	write_visualization_matches(in_real_image, in_synth_image, real_matched_pts, synth_matched_pts, "matches_2D");
	write_visualization_matches(in_real_image, in_synth_image, out_matched_image_points_real, out_matched_image_points_synth, "matches_with_3D_val");
}



bool Matching::is_distribution_good(
	const std::vector<cv::Point2f>& image_points,
	const cv::Size& image_size,
	int grid_rows,
	int grid_cols,
	int min_pts_per_cell)
{
	std::vector<std::vector<int>> cell_counts(grid_rows, std::vector<int>(grid_cols, 0));

	// Determine width / height of each cell
	float cell_width = static_cast<float>(image_size.width) / grid_cols;
	float cell_height = static_cast<float>(image_size.height) / grid_rows;

	// count points in cell
	for (const auto& point : image_points) {
		int col = std::min(static_cast<int>(point.x / cell_width), grid_cols - 1);
		int row = std::min(static_cast<int>(point.y / cell_height), grid_rows - 1);
		cell_counts[row][col]++;
	}

	// check if each cell contains enough points
	for (const auto& row : cell_counts) {
		for (int count : row) {
			if (count < min_pts_per_cell) {
				return false; // distribution not sufficent
			}
		}
	}
	return true; // distribution ok
}



bool Matching::run_calibration(
	std::vector<std::vector<cv::Point3f>> object_pts,
	std::vector<std::vector<cv::Point2f>> image_pts,
	cv::Size& image_size,
	cv::Mat& camera_matrix,
	cv::Mat& dist_coeffs,
	bool use_fisheye_model)
{

	cv::TermCriteria termCrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 300, DBL_EPSILON);
	
	// Run calibration and calculate RMS
	double rms;
	if (use_fisheye_model) {
		cv::Mat _rvecs, _tvecs;
		int flags = 
			cv::fisheye::CALIB_USE_INTRINSIC_GUESS + 
			cv::fisheye::CALIB_FIX_SKEW + 
			cv::fisheye::CALIB_FIX_K4 +
			cv::fisheye::CALIB_FIX_K3 + 
			cv::fisheye::CALIB_FIX_K2 + 
			cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;
		rms = cv::fisheye::calibrate(object_pts, image_pts, image_size, camera_matrix, dist_coeffs,
			_rvecs, _tvecs, flags, termCrit);
	}
	else {
		std::vector<cv::Mat> tvecs = {};
		std::vector<cv::Mat> rvecs = {};
		int flags = 
			cv::CALIB_USE_INTRINSIC_GUESS + 
			cv::CALIB_FIX_ASPECT_RATIO;
		rms = cv::calibrateCamera(object_pts, image_pts, image_size, camera_matrix, dist_coeffs,
			rvecs, tvecs, flags, termCrit);
	}

	_logfile->append("Camera calibration, rms: " + std::to_string(rms));

	// Check calibration results: verify that parameters are within valid ranges
	bool ok = checkRange(camera_matrix) && checkRange(dist_coeffs);

	// Return true if calibration is successful (parameters are valid), false otherwise
	return ok;
}



double Matching::space_resection(std::vector<cv::Point3d>& in_matched_object_points,
	 std::vector<cv::Point2d>& in_matched_image_points_real,
	 cv::Mat& true_image,
	 cv::Mat& camera_matrix,
	 cv::Mat& dist_coeffs,
	 cv::Mat& rvec,
	 cv::Mat& tvec,
	 cv::Mat& stdDev_In,
	 cv::Mat& stdDev_Ext,
	 Flags_resec in_flag,
	 bool ultra_wide_angle) {
	 
	 // Early return if both extrinsic and intrinsic parameters are fixed
	 if (in_flag == FIXED_EO_IO) {
		 _logfile->append(TAG + " Both EO and IO fixed.");
		 return -1;
	 }
	 
	 // Validate input parameters and data
	 if (camera_matrix.total() != 9 || dist_coeffs.empty() ||
		 in_matched_object_points.size() < 4 || in_matched_image_points_real.size() < 4) {
		 _logfile->append(TAG + ", invalid inputs.");
		 return -1;
	 }

	 // init inlier vector
	 std::vector<int> inliers;

	 // Optional: Set a deterministic random seed to make RANSAC reproducible
	 cv::theRNG().state = 42;

	 // Check if rvec is a 3x3 matrix
	 if (rvec.rows == 3 && rvec.cols == 3) {
		 cv::Rodrigues(rvec, rvec);
	 }

	 // Run solvePnPRansac for initial pose estimation and outlier filtering
	 // use AP3P which seems to be more robust against uncertainities in the IOP and EOP than LM-Solver but less accurate
	 if (!cv::solvePnPRansac(in_matched_object_points, in_matched_image_points_real,
		 camera_matrix, dist_coeffs, rvec, tvec, true,
		 100, 8.0f, 0.99, //default parameters
		 inliers, cv::SOLVEPNP_AP3P)) 
	 {
		 _logfile->append(TAG + "solvePnPRansac failed.");
		 return -1;
	 }
	 else {
		 _logfile->append(TAG + "solvePnPRansac, number of inliers: " + std::to_string(inliers.size()));
	 }

	 // Collect inliers
	 std::vector<cv::Point3f> object_points_ransac;
	 std::vector<cv::Point2f> image_points_real_ransac;
	 for (int i : inliers) {
		 image_points_real_ransac.push_back(in_matched_image_points_real[i]);
		 object_points_ransac.push_back(in_matched_object_points[i]);
	 }

	 // Optimize extrinsics or both intrinsic and extrinsic parameters
	 cv::Size imageSize = true_image.size();  // Store size in a variable
	 
	 if (in_flag ==!CALC_EO) {
		 // Check the distribution of points inside the image
		 if (!is_distribution_good(image_points_real_ransac, imageSize)) {
			 _logfile->append("Distribution of 2D-3D correspondences is not sufficient to calculate camera matrix and distortion coefficients.");
		 }
		 else {
			_logfile->append(TAG + " Optimizing IO");
			std::vector<std::vector<cv::Point3f>> obj_pts_vector = { object_points_ransac };
			std::vector<std::vector<cv::Point2f>> img_pts_vector = { image_points_real_ransac };
			run_calibration(obj_pts_vector, img_pts_vector, imageSize, camera_matrix, dist_coeffs, ultra_wide_angle);
		 }
	 }

	 // calculate reprojection error
	 // project 3D points to image
	 std::vector<cv::Point2f> projectedPoints;
	 cv::projectPoints(object_points_ransac, rvec, tvec, camera_matrix, dist_coeffs, projectedPoints);

	 // calculate error for each point
	 double totalError = 0.0;
	 for (size_t i = 0; i < image_points_real_ransac.size(); ++i) {
		 double error = cv::norm(image_points_real_ransac[i] - projectedPoints[i]);
		 totalError += error * error;  // Quadratischer Fehler
	 }

	 // calculate mean error
	 double repro_error = std::sqrt(totalError / image_points_real_ransac.size());
	 
	 // Ensure rvec and tvec are single-channel
	 if (rvec.channels() == 3 || tvec.channels() == 3) {
		 rvec = rvec.reshape(1);
		 tvec = tvec.reshape(1);
	 }
	 return repro_error;
}



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
	_logfile->append(TAG + "RansacTest, Final Number of Passed Points " + std::string(buf_pt1) + "," + std::string(buf_pt2) + "\n");

	// Return the final fundamental matrix
	return fundamental;
}


void Matching::image_points_3D_referencing(std::vector<std::vector<cv::Point2d>>& input_image_points, std::vector<cv::Point3d>& synth_pts_3D, cv::Mat& in_image_4_color, cv::Mat& camera_matrix, cv::Mat& dist_coeffs, cv::Mat& rvec_cc_orig_copy, cv::Mat& tvec_cc_orig_copy, double shift_x, double shift_y, double shift_z, bool export_pcl, std::vector<std::string> file_name_image_points) {

	std::stringstream log_statistics;

	Model model = Model(_logfile); // Initialize model with logging
	std::vector<cv::Vec3b> point_cloud_color;
	std::vector<cv::Point2d> image_coordinates_color;
	double distance_threshold_img_to_proj_img = 2.0; // pixels

	// Process each set of input image points separately
	for (size_t idx = 0; idx < input_image_points.size(); ++idx) {
		std::vector<cv::Point2d> img_pts_2D_undistort_normalized_coordinates;
		std::vector<cv::Point2d> img_pts_2D_undistort_image_coordinates;

		const auto& points_set = input_image_points[idx];

		if (!points_set.empty()) {
			// Undistort input 2D image points to normalized coordinates
			cv::undistortPoints(points_set, img_pts_2D_undistort_normalized_coordinates, camera_matrix, dist_coeffs);

			// Convert normalized coordinates back to image space using the camera matrix
			for (const cv::Point2d& p : img_pts_2D_undistort_normalized_coordinates) {
				img_pts_2D_undistort_image_coordinates.emplace_back(
					camera_matrix.at<double>(0, 0) * p.x + camera_matrix.at<double>(0, 2),
					camera_matrix.at<double>(1, 1) * p.y + camera_matrix.at<double>(1, 2)
				);
			}
		}
		else {
			_logfile->append(TAG + "no image points to reference provided for set " + std::to_string(idx) + ", will only color point cloud");
			continue; // Skip further processing if no points are provided in this set
		}

		// Project the synthetic 3D points to image space and retrieve colors for each point
		Model::ReferencedPoints referenced_points = model.get_color_for(synth_pts_3D, in_image_4_color, point_cloud_color, image_coordinates_color, 1.0, camera_matrix, dist_coeffs, rvec_cc_orig_copy, tvec_cc_orig_copy, points_set);

		// Export recolored 3D point cloud if required
		if (export_pcl && idx == 0) {
			_logfile->append(TAG + "---- export 3D point cloud ----");
			model.export_point_cloud_recolored(_working_dir_matching, synth_pts_3D, point_cloud_color, image_coordinates_color, shift_x, shift_y, shift_z);
		}

		// Check if 3D referencing succeeded for the current set
		if (!referenced_points.corresponding_3D_image_pts_from_point_cloud.empty()) {
			// Filter outliers based on distance threshold
			Matching::FilteredData filtered_data = filter_pts_by_distance(points_set, referenced_points.corresponding_2D_image_pts_from_point_cloud, referenced_points.corresponding_3D_image_pts_from_point_cloud, distance_threshold_img_to_proj_img);

			// Clone master image for drawing points
			cv::Mat copy_masterimage = in_image_4_color.clone();

			// Draw input image points on the image
			for (const cv::Point2d& p : points_set) {
				cv::circle(copy_masterimage, p, 5, cv::Scalar(255, 255, 0), -1); // Cyan circles for input points
			}

			// Draw projected image points on the image
			for (const cv::Point2d& p : filtered_data.image_data_projected) {
				cv::circle(copy_masterimage, p, 4, cv::Scalar(255, 0, 255), -1); // Magenta circles for projected points
			}

			// Save the output image with a unique identifier
			std::string output_image_filename = file_name_image_points[idx] + "_projected_" + std::to_string(idx) + ".png";
			cv::imwrite(fs::path(_working_dir_matching / output_image_filename).string(), copy_masterimage);

			// Export 3D referenced points to a text file with a unique identifier
			std::string output_text_filename = file_name_image_points[idx] + "_projected_" + std::to_string(idx) + ".txt";
			std::filesystem::path file_path = _working_dir_matching / output_text_filename;
			std::ofstream myfile(file_path);

			// Check if file is opened successfully
			if (!myfile.is_open()) {
				throw std::runtime_error("Could not open the file: " + file_path.string());
			}

			// Write the 3D points with shifts applied and corresponding IDs
			for (size_t i = 0; i < filtered_data.image_data_3D.size(); ++i) {
				myfile << std::fixed << std::setprecision(4)
					<< filtered_data.image_data_original_idx[i] << ","
					<< filtered_data.image_data_3D[i].x + shift_x << ","
					<< filtered_data.image_data_3D[i].y + shift_y << ","
					<< filtered_data.image_data_3D[i].z + shift_z << "\n";
			}
			myfile.close();

			// Log the count of referenced image points for the current set
			_logfile->append(TAG + "count referenced image points for set " + std::to_string(idx) + ": " + std::to_string(filtered_data.image_data_3D.size()));
		}
	}
}



Matching::FilteredData Matching::filter_pts_by_distance(
	const std::vector<cv::Point2d>& list1,
	const std::vector<cv::Point2d>& list2,
	const std::vector<cv::Point3d>& list2_3d,
	double distanceThreshold) 
{
	std::vector<cv::Point2d> filtered_list1;       // Filtered points from list1
	std::vector<cv::Point2d> filtered_list2;       // Filtered points from list2
	std::vector<cv::Point3d> filtered_list2_3d;    // Filtered 3D points corresponding to list2
	std::vector<int> filtered_idx;                 // Indices of points retained after filtering
	int counter = 0;

	// Loop through all pairs of 2D points in list1 and list2
	for (size_t i = 0; i < list1.size(); ++i) {
		const auto& point1 = list1[i];
		const auto& point2 = list2[i];
		const auto& point2_3D = list2_3d[i];

		// Check if the Euclidean distance between points is within the threshold
		if (euclidean_distance(point1, point2) < distanceThreshold) {
			filtered_idx.push_back(counter);              // Store index of the point
			filtered_list1.push_back(point1);             // Store point from list1
			filtered_list2.push_back(point2);             // Store point from list2
			filtered_list2_3d.push_back(point2_3D);       // Store corresponding 3D point
		}

		counter++;
	}
	return FilteredData{ filtered_idx, filtered_list1, filtered_list2, filtered_list2_3d };
}


void Matching::write_visualization_matches(cv::Mat& in_real_image, cv::Mat& in_synth_image, std::vector<cv::Point2d>& in_real_matches_draw, std::vector<cv::Point2d>& in_synth_matches_draw, std::string fileName) {

	#ifdef max
	#undef max
	#endif
	// Create a new image to combine both real and synthetic images side by side
	cv::Mat matchesImage = cv::Mat(
		std::max(in_synth_image.rows, in_real_image.rows),    // Choose the larger height between the two images
		in_synth_image.cols + in_real_image.cols,             // Sum of the widths of both images
		in_synth_image.type(),                                // Set the image type (same as the synthetic image)
		cv::Scalar(0, 0, 0));                                 // Initialize with black background

	_logfile->append(TAG + "Visualizing matches between real and synthetic images");

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
		cv::Point2d point_real = cv::Point2d(in_real_matches_draw[i].x, in_real_matches_draw[i].y);
		cv::Point2d point_synth = cv::Point2d(in_synth_matches_draw[i].x, in_synth_matches_draw[i].y);
		point_synth.x += in_real_image.cols; // Shift synthetic points by real image width for correct positioning

		// Draw circles around the keypoints
		circle(matchesImage, point_real, 3, matchColor, 1);  // Real image keypoints in red
		circle(matchesImage, point_synth, 3, matchColor, 1); // Synthetic image keypoints in red

		// Draw lines connecting the keypoints between the real and synthetic images
		cv::line(matchesImage, point_real, point_synth, matchColor, 2, 8, 0);
	}

	// Save the final image with the drawn matches
	cv::imwrite(fs::path(_working_dir_matching / (fileName + ".jpg")).string(), matchesImage);
}


void Matching::write_camera_calibration_statistics(cv::Mat& in_camera_matrix, cv::Mat& in_dist_coeffs, cv::Mat& in_rvec, cv::Mat& in_tvec, cv::Mat& in_StdDev_IntO, cv::Mat& in_StdDev_ExtO, cv::Mat& in_PerViewErrors, double in_pix_size) {
	// Add a blank line for formatting in the log file and indicate the start of camera calibration results
	_logfile->append("");
	_logfile->append(TAG + "---- camera calibration results ----");

	// Access the data of the intrinsic camera matrix and standard deviations
	// ocv_docu: A(0, 0) = param[0]; A(1, 1) = param[1]; A(0, 2) = param[2]; A(1, 2) = param[3]; std::copy(param + 4, param + 4 + 14, k);
	double* camera_matrix_data = (double*)(in_camera_matrix.data);
	double* stdDev_In_data = (double*)(in_StdDev_IntO.data);

	// Log the intrinsic parameters of the camera (focal lengths, optical center) in both pixels and mm
	_logfile->append("\n" + TAG + "------------- intrinsics -------------");
	_logfile->append(TAG + "fx: " + std::to_string(camera_matrix_data[0]) + " [px]/ " + std::to_string(camera_matrix_data[0] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[0]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[0] * in_pix_size), 5);
	_logfile->append(TAG + "fy: " + std::to_string(camera_matrix_data[4]) + " [px]/ " + std::to_string(camera_matrix_data[4] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[1]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[1] * in_pix_size), 5);
	_logfile->append(TAG + "cx: " + std::to_string(camera_matrix_data[2]) + " [px]/ " + std::to_string(camera_matrix_data[2] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[2]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[2] * in_pix_size), 5);
	_logfile->append(TAG + "cy: " + std::to_string(camera_matrix_data[5]) + " [px]/ " + std::to_string(camera_matrix_data[5] * in_pix_size) + "[mm], std_dev [px]: +/-" + std::to_string(stdDev_In_data[3]) + ", [mm]: +/-" + std::to_string(stdDev_In_data[3] * in_pix_size), 5);

	// Access the distortion coefficients and log them along with their standard deviations
	double* dist_coeffs_data = (double*)(in_dist_coeffs.data);
	_logfile->append("\n" + TAG + "------------- distortion coefficients -------------");
	_logfile->append(TAG + "k1: " + std::to_string(dist_coeffs_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[4]), 8);
	_logfile->append(TAG + "k2: " + std::to_string(dist_coeffs_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[5]), 8);
	_logfile->append(TAG + "p1: " + std::to_string(dist_coeffs_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[6]), 8);
	_logfile->append(TAG + "p2: " + std::to_string(dist_coeffs_data[3]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[7]), 8);
	_logfile->append(TAG + "k3: " + std::to_string(dist_coeffs_data[4]) + ", std_dev: +/-" + std::to_string(stdDev_In_data[8]), 8);

	// Access the rotation and translation vectors along with their standard deviations
	double* rvec_cc_1ch_data = (double*)(in_rvec.data);
	double* tvec_cc_1ch_data = (double*)(in_tvec.data);
	double* stdDev_Ext_data = (double*)(in_StdDev_ExtO.data);
	_logfile->append("\n" + TAG + "------------- extrinsics -------------");
	_logfile->append(TAG + "rotV r0: " + std::to_string(rvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[0]), 5);
	_logfile->append(TAG + "rotV r1: " + std::to_string(rvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[1]), 5);
	_logfile->append(TAG + "rotV r2: " + std::to_string(rvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[2]), 5);
	_logfile->append(TAG + "transV t0: " + std::to_string(tvec_cc_1ch_data[0]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[3]), 5);
	_logfile->append(TAG + "transV t1: " + std::to_string(tvec_cc_1ch_data[1]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[4]), 5);
	_logfile->append(TAG + "transV t2: " + std::to_string(tvec_cc_1ch_data[2]) + ", std_dev: +/-" + std::to_string(stdDev_Ext_data[5]), 5);

	// Access the per-view errors and log the reprojection error
	double* perViewErrors_data = (double*)(in_PerViewErrors.data);
	_logfile->append("\n" + TAG + "------------- additional information -------------");
	_logfile->append(TAG + "per-view error: " + std::to_string(perViewErrors_data[0]), 4);
}


void Matching::write_and_print_log_statistics(std::stringstream& log_statistics) const {
	// Construct the file path in a cross-platform way
	std::filesystem::path statsFilePath = std::filesystem::path(_working_dir_matching) / "image_points_projected_stats.txt";

	// Open the file to store log statistics
	std::ofstream stats_file(statsFilePath);
	if (!stats_file) {
		std::cerr << "Failed to open file: " << statsFilePath << std::endl;
		return;
	}

	std::string line;
	// Iterate through each line of the log statistics stringstream
	while (std::getline(log_statistics, line)) {
		// Print each line to the console
		std::cout << line << std::endl;
		// Write each line to the file
		stats_file << line << std::endl;
	}

	// Close the file stream after writing
	stats_file.close();
}


void Matching::write_corresponding_points_to_file(
	const std::vector<cv::Point3d>& object_points_3D,
	const std::vector<cv::Point2d>& image_points_2D_real,
	const std::vector<cv::Point2d>& image_points_2D_synth)
{
	// Construct paths using std::filesystem
	std::filesystem::path file3DPath = std::filesystem::path(_working_dir_matching) / "corresp_points_3D.txt";
	std::filesystem::path file2DRealPath = std::filesystem::path(_working_dir_matching) / "corresp_points_2D_real.txt";
	std::filesystem::path file2DSynthPath = std::filesystem::path(_working_dir_matching) / "corresp_points_2D_synth.txt";

	// Open files and check for success
	std::ofstream myfile3D(file3DPath), myfile2D_real(file2DRealPath), myfile2D_synth(file2DSynthPath);
	if (!myfile3D || !myfile2D_real || !myfile2D_synth) {
		_logfile->append(TAG + "Failed to open one or more output files. Aborting.");
		return;
	}

	// Check if vector sizes are equal
	if (object_points_3D.size() != image_points_2D_real.size() ||
		object_points_3D.size() != image_points_2D_synth.size())
	{
		_logfile->append(TAG + "Number of 3D & 2D points do not match. Aborting.");
		return;
	}

	// Write each point to its corresponding file
	for (size_t counter = 0; counter < object_points_3D.size(); ++counter) {
		// Write 3D points
		myfile3D << "1 " << counter << " "
			<< object_points_3D[counter].x << " "
			<< object_points_3D[counter].y << " "
			<< object_points_3D[counter].z << std::endl;

		// Write real 2D points
		myfile2D_real << "1 " << counter << " "
			<< image_points_2D_real[counter].x << " "
			<< image_points_2D_real[counter].y << std::endl;

		// Write synthetic 2D points
		myfile2D_synth << "1 " << counter << " "
			<< image_points_2D_synth[counter].x << " "
			<< image_points_2D_synth[counter].y << std::endl;
	}

	// Log success messages
	_logfile->append(TAG + "Successfully written: corresp_points_3D.txt");
	_logfile->append(TAG + "Successfully written: corresp_points_2D_real.txt");
	_logfile->append(TAG + "Successfully written: corresp_points_2D_synth.txt");
}