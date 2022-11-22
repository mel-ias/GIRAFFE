
#include"opencv2\opencv.hpp"
#include"kdtree.h"
#include"LogfilePrinter.h"


class Modell_OCV
{

public:
	// C'tor, D'tor
	Modell_OCV(LogFile* _logfile); 
	~Modell_OCV();

	// retrieves color values for given 3d coordinates from image file; stdval 'bool fix_aspect_ratio = 1.0'
	std::vector<cv::Point3d> Modell_OCV::getColorFor(std::vector<cv::Point3d>& point_cloud, cv::Mat& image_for_color, std::vector<cv::Vec3b>& point_cloud_colors, std::vector<cv::Point2d>& image_coords_colors, bool fix_aspect_ratio, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, cv::Mat& rvec, cv::Mat& tvec, std::vector<cv::Point2d>& waterlinePoints);
	// output recolored point cloud
	void export_point_cloud_recolored(std::string workingDirectory, std::string wD_name, std::vector<cv::Point3d>& point_cloud, std::vector<cv::Vec3b>& point_cloud_colors, std::vector<cv::Point2d>& image_coords_colors, double shifter_x, double shifter_y) ;

private:
	LogFile* logfile;
	const std::string TAG = "Modell_OCV:\t\t";
};