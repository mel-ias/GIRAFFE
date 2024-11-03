#include "Model.h"


// C'tor
Model::Model(LogFile* _logfile) {
	logfile = _logfile;
	logfile->append("");
	logfile->append(TAG + "Initialisation of Model");
}
	



std::vector<int> Model::findNearestNeighbors(
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

	// Query KD-tree for each image_point
	for (size_t i = 0; i < image_points.size(); ++i) {
		std::vector<float> query = { static_cast<float>(image_points[i].x), static_cast<float>(image_points[i].y) };
		std::vector<int> knn_indices(1);
		std::vector<float> knn_dists(1);

		// Perform knnSearch with k=1 (find the nearest neighbor)
		kdtree.knnSearch(query, knn_indices, knn_dists, 1);

		// Save the index of the nearest neighbor
		indices[i] = knn_indices[0];
	}

	return indices;
}





Model::ReferencedPoints Model::getColorFor(std::vector<cv::Point3d>& point_cloud,
	const cv::Mat& image_for_color,
	std::vector<cv::Vec3b>& point_cloud_colors,
	std::vector<cv::Point2d>& image_coords_colors,
	const bool fix_aspect_ratio,
	const cv::Mat& cameraMatrix,
	const cv::Mat& distCoeffs,
	const cv::Mat& rvec,
	const cv::Mat& tvec,
	const std::vector<cv::Point2d>& image_points) {

	std::vector<cv::Point2d> image_pixels;
	std::vector<std::pair<cv::Point3d, int>> image_points_projected; // Vector to store pairs of projected points and their indices
	std::unordered_set<std::string> unique_points; // To track unique projected points

	// Project the point cloud into the image space
	cv::projectPoints(point_cloud, rvec, tvec, cameraMatrix, distCoeffs, image_pixels, cv::noArray(), fix_aspect_ratio);

	std::cout << "number of pcl points: " << point_cloud.size() << ", number of projected pts: " << image_pixels.size() << std::endl;

	// Iterate through all pixel coordinates and get the color from the image
	for (size_t i = 0; i < image_pixels.size(); ++i) {
		cv::Point2d pixel = image_pixels[i];

		// Check if the pixel is within image bounds
		if (pixel.x >= 0 && pixel.y >= 0 && pixel.x < image_for_color.cols && pixel.y < image_for_color.rows) {
			// Fetch the color from the image at the projected point
			point_cloud_colors.push_back(image_for_color.at<cv::Vec3b>(cv::Point(pixel.x, pixel.y)));
			image_coords_colors.push_back(pixel); // Save the valid image coordinates
		}
		else {
			// Assign a default color (black) if out of bounds
			point_cloud_colors.push_back(cv::Vec3b(0, 0, 0));
			image_coords_colors.push_back(cv::Point2d(-1, -1)); // Invalid coordinates
		}
	}

	Model::ReferencedPoints referenced_pts;
	// If no waterline points are provided, return immediately
	if (image_points.empty()) {
		return referenced_pts;
	}

	// image_pixels -> projeceted points from point cloud
	// image_points -> original 2D points
	// Find nearest neighbors
	std::vector<int> indices = findNearestNeighbors(image_points, image_pixels);

	// Print results
	int img_pts_counter = 0;
	for (int ind : indices) {
		referenced_pts.original_2D_image_pts.push_back(image_points[img_pts_counter]);
		referenced_pts.corresponding_3D_image_pts_from_point_cloud.push_back(point_cloud[ind]);
		referenced_pts.corresponding_2D_image_pts_from_point_cloud.push_back(image_pixels[ind]);
		img_pts_counter++;	
	}
	return referenced_pts;
}



void Model::export_point_cloud_recolored(
	const std::string& workingDirectory,
	const std::string& wD_name,
	const std::vector<cv::Point3d>& point_cloud,
	const std::vector<cv::Vec3b>& point_cloud_colors,
	const std::vector<cv::Point2d>& image_coords_colors,
	const double shifter_x,
	const double shifter_y,
	const double shifter_z) {

	// Open output file stream
	std::ofstream outStream(workingDirectory + wD_name + "_pcl.txt");

	// Check if the file opened successfully
	if (!outStream.is_open()) {
		throw std::runtime_error("Could not open output file for writing.");
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


