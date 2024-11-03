


#include"LogfilePrinter.h"


#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <tuple>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <iomanip>
#include <string>


class Model
{

public:
	Model(LogFile* _logfile);

	~Model() = default;  // Destruktor muss jetzt nichts manuell freigeben


	struct ReferencedPoints {   // Structure declaration
		std::vector<cv::Point2d> original_2D_image_pts; 
		std::vector<cv::Point2d> corresponding_2D_image_pts_from_point_cloud;
		std::vector<cv::Point3d> corresponding_3D_image_pts_from_point_cloud;
	}; // End the structure with a semicolon


	std::vector<int> findNearestNeighbors(
		const std::vector<cv::Point2d>& image_points,
		const std::vector<cv::Point2d>& image_pixels);


	
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
	ReferencedPoints getColorFor(std::vector<cv::Point3d>& point_cloud,
		const cv::Mat& image_for_color,
		std::vector<cv::Vec3b>& point_cloud_colors,
		std::vector<cv::Point2d>& image_coords_colors,
		const bool fix_aspect_ratio,
		const cv::Mat& cameraMatrix,
		const cv::Mat& distCoeffs,
		const cv::Mat& rvec,
		const cv::Mat& tvec,
		const std::vector<cv::Point2d>& image_points);
	

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
	void export_point_cloud_recolored(
		const std::string& workingDirectory,
		const std::string& wD_name,
		const std::vector<cv::Point3d>& point_cloud,
		const std::vector<cv::Vec3b>& point_cloud_colors,
		const std::vector<cv::Point2d>& image_coords_colors,
		const double shifter_x,
		const double shifter_y,
		const double shifter_z);
private:
	LogFile* logfile;
	const std::string TAG = "Modell_OCV:\t\t";
};