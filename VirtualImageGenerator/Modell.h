//
//#include"opencv2\opencv.hpp"
//#include"kdtree.h"
//#include"LogfilePrinter.h"
//
//
//
//class Modell
//{
//
//
//	struct Waterline3dPoints {
//		cv::Point2d imagePoint1;
//		cv::Point3d objectPoint1;
//
//		Waterline3dPoints(cv::Point2d _imagePoints, cv::Point3d _objectPoints) {
//			imagePoint1 = _imagePoints;
//			objectPoint1 = _objectPoints;
//		}
//	};
//
//	struct Quaternion {
//		double a, b, c, d;
//
//		Quaternion() {
//
//		}
//
//		Quaternion(double _a, double _b, double _c, double _d) {
//			a = _a;
//			b = _b;
//			c = _c;
//			d = _d;
//		}
//	};
//
//public:
//	
//	Modell(LogFile* logfile, cv::Mat& transfMat, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, int imageWidth, int imageHeight, double pixelGr, std::vector<cv::Point2d>* waterlinePoints);
//	
//	//Modell(double* projC, double* quat, cv::Mat& distCoeffs, int imageWidth, int imageHeight, double pixelGr, std::vector<cv::Point2d>* waterlinePoints);
//
//
//
//	~Modell(); 
//
//
//	//Modell(cv::Mat* transfMat, cv::Mat* cameraMatrix, cv::Mat* distCoeffs, int imageWidth, int imageHeight, double pixelGr, std::vector<cv::Point2d>* waterlinePoints);
//
//	
//	cv::Mat& Modell::getTransCameraMatrix() { return trans_cameraMatrix;  } //int_O brown
//	cv::Mat& Modell::getTransTransformationMatrix() { return trans_transformationMatrix; } //int_O brown
//	cv::Mat& Modell::getTransDistortionCoeffs() { return trans_distCoeffs; } //int_O brown
//	std::vector<cv::Point3d>* Modell::getWaterlinepointsProjected() { return waterlinePoints_projected; }
//
//	/*void Modell::berechneNachbarnMatching(
//
//		const std::vector<Vek2d>& image_pt_in,//punkte
//		const std::vector<Vek2d>& image_pt_2match, //punkte3d
//		std::vector<cv::Point2d>& matchedPts,
//		float nachbarDistanz = 1.0);
//	*/
//	//void setMasterImage(cv::Mat _masterImage) {
//	//	masterImage = _masterImage;
//	//}
//	//
//	void initialisePointCloudProjection();
//
//
//
//
//	// retrieves color values for given 3d coordinates from image file
//	void getColorFor(cv::Point3d& v, cv::Mat& realImage, cv::Vec3b& color);
//	void getImageCoordinates(cv::Point3d& v, cv::Mat &image, cv::Point2d &point);
//
//
//	double get_ck() { return ck; }
//	cv::Point2d get_principlePoint() { return cv::Point2d(x0, y0); }
//	cv::Vec3f get_EulerAngles() { return eulerAngles; }
//	cv::Point3d get_ProjC() { return cv::Point3d(Xo, Yo, Zo); }
//
//
//	cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
//	bool isRotationMatrix(cv::Mat &R);
//
//private:
//	LogFile* logfile;
//	const std::string TAG = "Modell:\t\t";
//
//	cv::Mat transformationsMatrix;
//	cv::Mat cameraMatrix; // fx = fy! 
//	cv::Mat distCoeffs; //k1 k2  p1 p2 k3
//	//std::vector<cv::Point2d>* waterlinePoints;
//	cv::Mat masterImage;
//
//	uint imageWidth;
//	uint imageHeight;
//	double pixelGr;
//
//	// orientation parameters
//	double Xo, Yo, Zo, x0, y0, ck, omega, phi, kappa;
//	double A1, A2, A3, A4 = 0, B1, B2, B3 = 0, B4 = 0, C1 = 0, C2 = 0;
//
//	double x, y, r0 = 0;
//
//	// Quaternionen
//	double a,b,c,d;
//
//	// rotation matrix elements
//	double r11, r21, r31;
//	double r12, r22, r32;
//	double r13, r23, r33;
//
//	//cv::Mat mImage;
//
//	// sensor size in x and y (mm)
//	double pixSizeX, pixSizeY;
//
//	// minimum extension of camera plane 
//	double xMin, yMin;
//
//	// corner points of 3d camera object for rendering
//	//std::vector<double[3]> corner;
//
//
//	cv::Mat trans_transformationMatrix;
//	cv::Mat trans_cameraMatrix;
//	cv::Mat trans_distCoeffs;
//
//	std::vector<cv::Point2d>* waterlinePoints;
//	std::vector<cv::Point3d>* waterlinePoints_projected;
//
//	cv::Vec3f eulerAngles;
//	
//	void conversion();
//
//	
//
//};