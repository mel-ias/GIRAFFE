
#include"opencv2\opencv.hpp"
#include"kdtree.h"
#include"LogfilePrinter.h"



class Modell_OCV
{


	struct Waterline3dPoints {
		cv::Point2d imagePoint1;
		cv::Point3d objectPoint1;

		Waterline3dPoints(cv::Point2d _imagePoints, cv::Point3d _objectPoints) {
			imagePoint1 = _imagePoints;
			objectPoint1 = _objectPoints;
		}
	};

	

public:

	Modell_OCV(LogFile* _logfile, cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs, cv::Mat& _rvec, cv::Mat& _tvec, std::vector<cv::Point2d>& _waterlinePoints);	

	~Modell_OCV();

	std::vector<cv::Point3d>* get_water_line_points_back_projected_3D() { 
		return &waterlinePoints_projected; 
	}


	// retrieves color values for given 3d coordinates from image file
	void getColorFor(std::vector<cv::Point3d>& point_cloud, cv::Mat& image_for_color, std::vector<cv::Vec3b> & point_cloud_colors, bool fix_aspect_ratio = 1.0);

	void print_point_cloud_recolored(std::string workingDirectory, std::vector<cv::Point3d>& point_cloud, std::vector<cv::Vec3b>& point_cloud_colors, double shifter_x = 0, double shifter_y = 0) ;

private:
	LogFile* logfile;
	const std::string TAG = "Modell_OCV:\t\t";

	cv::Mat tvec, rvec, cameraMatrix, distCoeffs;  //rvec vector! no matrix!
	// fx = fy! //k1 k2  p1 p2 k3

	cv::Mat masterImage;
	
	std::vector<cv::Point2d> waterlinePoints;
	std::vector<cv::Point3d> waterlinePoints_projected;



};