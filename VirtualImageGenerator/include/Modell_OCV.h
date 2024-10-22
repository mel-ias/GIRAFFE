


#include"LogfilePrinter.h"


#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <tuple>
#include <vector>
#include <unordered_set>
#include <fstream>
#include <iomanip>
#include <string>



class Modell_OCV
{

public:
	Modell_OCV(LogFile* _logfile);

	~Modell_OCV();

	std::vector<std::pair<cv::Point3d, int>> getColorFor(std::vector<cv::Point3d>& point_cloud,
		cv::Mat& image_for_color,
		std::vector<cv::Vec3b>& point_cloud_colors,
		std::vector<cv::Point2d>& image_coords_colors,
		bool fix_aspect_ratio,
		cv::Mat& cameraMatrix,
		cv::Mat& distCoeffs,
		cv::Mat& rvec,
		cv::Mat& tvec,
		std::vector<cv::Point2d>& waterlinePoints);
	
	void export_point_cloud_recolored(
		const std::string& workingDirectory,
		const std::string& wD_name,
		const std::vector<cv::Point3d>& point_cloud,
		const std::vector<cv::Vec3b>& point_cloud_colors,
		const std::vector<cv::Point2d>& image_coords_colors,
		double shifter_x,
		double shifter_y,
		double shifter_z);
private:
	LogFile* logfile;
	const std::string TAG = "Modell_OCV:\t\t";
};