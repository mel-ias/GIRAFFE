#include "Modell_OCV.h"


// C'tor
Modell_OCV::Modell_OCV(LogFile* _logfile) {
	logfile = _logfile;
	logfile->append("");
	logfile->append(TAG + "---- initialisation open computer vision modell ----");
}

// D'tor
Modell_OCV::~Modell_OCV() {
}


/**
 * @brief Projects a point cloud onto an image and retrieves colors from the image.
 *
 * This function projects 3D points from a given point cloud into 2D image coordinates
 * using the provided camera parameters. It then extracts the color values from the
 * specified image for the projected points and associates them with a list of
 * waterline points.
 *
 * @param point_cloud A vector of 3D points representing the point cloud.
 * @param image_for_color The image from which color values will be extracted.
 * @param point_cloud_colors A reference to a vector where the colors corresponding
 *        to the projected points will be stored.
 * @param image_coords_colors A reference to a vector where the valid image
 *        coordinates of the projected points will be stored.
 * @param fix_aspect_ratio A boolean flag indicating whether to maintain the aspect
 *        ratio during the projection.
 * @param cameraMatrix The intrinsic camera matrix used for projecting the points.
 * @param distCoeffs The distortion coefficients for correcting the image.
 * @param rvec The rotation vector for the camera pose.
 * @param tvec The translation vector for the camera pose.
 * @param image_points A vector of 2D points representing the image points
 *        to be matched with the projected points.
 *
 * @return A vector of pairs, where each pair contains a projected 3D point and
 *         the index of the corresponding image point.
 */
std::vector<std::pair<cv::Point3d, int>> Modell_OCV::getColorFor(std::vector<cv::Point3d>& point_cloud,
	cv::Mat& image_for_color,
	std::vector<cv::Vec3b>& point_cloud_colors,
	std::vector<cv::Point2d>& image_coords_colors,
	bool fix_aspect_ratio,
	cv::Mat& cameraMatrix,
	cv::Mat& distCoeffs,
	cv::Mat& rvec,
	cv::Mat& tvec,
	std::vector<cv::Point2d>& image_points) {

	std::vector<cv::Point2d> image_pixels;
	std::vector<std::pair<cv::Point3d, int>> image_points_projected; // Vector to store pairs of projected points and their indices
	std::unordered_set<std::string> unique_points; // To track unique projected points

	// Project the point cloud into the image space
	cv::projectPoints(point_cloud, rvec, tvec, cameraMatrix, distCoeffs, image_pixels, cv::noArray(), fix_aspect_ratio);

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

	// If no waterline points are provided, return immediately
	if (image_points.empty()) {
		return image_points_projected;
	}

	// Prepare to find nearest neighbors using FLANN for the waterline points
	std::vector<cv::Point2f> image_points_to_search(image_pixels.begin(), image_pixels.end());
	cv::flann::KDTreeIndexParams indexParams;
	cv::flann::Index kdtree(cv::Mat(image_points_to_search).reshape(1), indexParams);

	// Search nearest neighbors for each waterline point
	for (size_t wl_index = 0; wl_index < image_points.size(); ++wl_index) {
		const auto& p_wl = image_points[wl_index];
		std::vector<float> query = { static_cast<float>(p_wl.x), static_cast<float>(p_wl.y) };
		std::vector<int> indices;
		std::vector<float> dists;

		// Find the nearest neighbor in the image pixels
		kdtree.knnSearch(query, indices, dists, 1);

		if (!indices.empty()) {
			const auto& projected_point = point_cloud[indices[0]];

			// Create a unique identifier for the point
			std::string unique_key = std::to_string(projected_point.x) + "_" +
				std::to_string(projected_point.y) + "_" +
				std::to_string(projected_point.z);

			// Check if the point has already been added
			if (unique_points.find(unique_key) == unique_points.end()) {
				// If not, add the point to the set and the output vector
				unique_points.insert(unique_key);
				image_points_projected.emplace_back(projected_point, wl_index); // Store the projected point and the corresponding image point index
			}
		}
	}
	return image_points_projected;
}



/**
 * @brief Exports a recolored point cloud to a text file with optional coordinate shifting.
 *
 * This function saves the 3D points of a point cloud along with their associated colors
 * and image coordinates to a text file. The coordinates can be shifted by specified
 * amounts along the X, Y, and Z axes. Only points with non-zero colors are included in
 * the output.
 *
 * @param workingDirectory The directory where the output file will be saved.
 * @param wD_name The base name for the output file (without extension).
 * @param point_cloud A vector of 3D points representing the point cloud.
 * @param point_cloud_colors A vector of colors corresponding to each point in the
 *        point cloud (in BGR format).
 * @param image_coords_colors A vector of 2D image coordinates corresponding to each
 *        point in the point cloud.
 * @param shifter_x The amount to shift the X coordinates of the points.
 * @param shifter_y The amount to shift the Y coordinates of the points.
 * @param shifter_z The amount to shift the Z coordinates of the points.
 */
void Modell_OCV::export_point_cloud_recolored(
	const std::string& workingDirectory,
	const std::string& wD_name,
	const std::vector<cv::Point3d>& point_cloud,
	const std::vector<cv::Vec3b>& point_cloud_colors,
	const std::vector<cv::Point2d>& image_coords_colors,
	double shifter_x,
	double shifter_y,
	double shifter_z) {

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


