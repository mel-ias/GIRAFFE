#include "Model.h"


Model::Model(LogFile* _logfile) {
	logfile = _logfile;
	logfile->append("");
	logfile->append(TAG + "Initialisation of Model");
}
	

std::vector<int> Model::find_knn(
	const std::vector<cv::Point2d>& image_points, 
	const std::vector<cv::Point2d>& image_pixels) {

	// Output array of matching indices
	std::vector<int> indices(image_points.size());

	// Prepare data for KD-tree by converting image_pixels to a single-channel Mat (type CV_32F)
	cv::Mat image_pixels_mat(image_pixels.size(), 2, CV_32F);
	for (size_t i = 0; i < image_pixels.size(); ++i) {
		image_pixels_mat.at<float>(i, 0) = static_cast<float>(image_pixels[i].x);
		image_pixels_mat.at<float>(i, 1) = static_cast<float>(image_pixels[i].y);
	}

	// Create KD-tree using FLANN with 2D points
	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree(image_pixels_mat, indexParams);

	// For each image point, find the nearest neighbor in the KD-tree
	for (size_t i = 0; i < image_points.size(); ++i) {
		std::vector<float> query = { static_cast<float>(image_points[i].x), static_cast<float>(image_points[i].y) };
		std::vector<int> knn_indices(1);
		std::vector<float> knn_dists(1);

		// Perform knnSearch with k=1 to find the closest neighbor
		kdtree.knnSearch(query, knn_indices, knn_dists, 1);

		// Store the nearest neighbor index
		indices[i] = knn_indices[0]; 
	}

	return indices; // Return the array of nearest neighbour indices
}


Model::ReferencedPoints Model::get_color_for(
	std::vector<cv::Point3d>& point_cloud, // 3D point cloud to project
	const cv::Mat& image_for_color, // Image from which to retrieve color
	std::vector<cv::Vec3b>& point_cloud_colors, // Vector to store colors of projected points
	std::vector<cv::Point2d>& image_coords_colors, // Vector to store valid image coordinates
	const bool fix_aspect_ratio, // Flag to fix aspect ratio (fx=fy) 
	const cv::Mat& cameraMatrix, 
	const cv::Mat& distCoeffs, 
	const cv::Mat& rvec,
	const cv::Mat& tvec,
	const std::vector<cv::Point2d>& image_points) // 2D image points to find nearest neighbors for
{

	std::vector<cv::Point2d> image_pixels; // projected 2d coordinates of the point cloud in image space
	std::vector<std::pair<cv::Point3d, int>> image_points_projected; // Projected points with indices
	std::unordered_set<std::string> unique_points; // Track unique projected points for debugging or filtering

	// Project 3D point cloud into the image coordinates
	cv::projectPoints(point_cloud, rvec, tvec, cameraMatrix, distCoeffs, image_pixels, cv::noArray(), fix_aspect_ratio);

	// Retrieve color from the image at each projected point
	for (size_t i = 0; i < image_pixels.size(); ++i) {
		cv::Point2d pixel = image_pixels[i];

		// Check if the projected pixel is within the bounds of the image
		if (pixel.x >= 0 && pixel.y >= 0 && pixel.x < image_for_color.cols && pixel.y < image_for_color.rows) {
			// Fetch color at the projected pixel
			point_cloud_colors.push_back(image_for_color.at<cv::Vec3b>(cv::Point(pixel.x, pixel.y)));
			image_coords_colors.push_back(pixel); // Save valid coordinates for reference
		}
		else {
			// If out of bounds, assign a default color (e.g., black) and mark coordinates as invalid
			point_cloud_colors.push_back(cv::Vec3b(0, 0, 0));
			image_coords_colors.push_back(cv::Point2d(-1, -1)); // Mark invalid coordinates
		}
	}
	
	Model::ReferencedPoints referenced_pts; // Structure to store matching points

	// If no original 2D points are provided, return immediately
	if (image_points.empty()) {
		return referenced_pts;
	}

	// Find nearest neighbors between original 2D points and projected points
	std::vector<int> indices = find_knn(image_points, image_pixels);

	// Collect matched points
	int img_pts_counter = 0;
	for (int ind : indices) {
		referenced_pts.original_2D_image_pts.push_back(image_points[img_pts_counter]);
		referenced_pts.corresponding_3D_image_pts_from_point_cloud.push_back(point_cloud[ind]);
		referenced_pts.corresponding_2D_image_pts_from_point_cloud.push_back(image_pixels[ind]);
		img_pts_counter++;	
	}
	return referenced_pts; // Return the structure containing matched points and their references
}


void Model::export_point_cloud_recolored(
	const fs::path& workingDirectory,
	const std::vector<cv::Point3d>& point_cloud,
	const std::vector<cv::Vec3b>& point_cloud_colors,
	const std::vector<cv::Point2d>& image_coords_colors,
	const double shifter_x,
	const double shifter_y,
	const double shifter_z) {

	// Build the full path for the output file
	fs::path outputFilePath = workingDirectory / "recolored_pcl.txt";

	// Open output file stream
	std::ofstream outStream(outputFilePath);

	// Check if the file opened successfully
	if (!outStream.is_open()) {
		throw std::runtime_error("Could not open output file for writing: " + outputFilePath.string());
	}

	// Output point cloud to file with coordinate shifting
	for (size_t i = 0; i < point_cloud.size(); ++i) {
		const cv::Point3d& p = point_cloud[i];
		const cv::Vec3b& c = point_cloud_colors[i];
		const cv::Point2d& imgP = image_coords_colors[i];

		// Check for non-zero color values
		if (c[0] > 0 || c[1] > 0 || c[2] > 0) {
			outStream << std::fixed << std::setprecision(4)
				<< p.x + shifter_x << ","
				<< p.y + shifter_y << ","
				<< p.z + shifter_z << ","
				<< static_cast<unsigned int>(c[2]) << ","
				<< static_cast<unsigned int>(c[1]) << ","
				<< static_cast<unsigned int>(c[0]) << ","
				<< std::fixed << imgP.x << ","
				<< std::fixed << imgP.y << "\n";
		}
	}
	outStream.close();
	logfile->append(TAG + "saved " + std::to_string(point_cloud.size()) + " points.");
}