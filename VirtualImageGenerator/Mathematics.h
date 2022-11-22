#ifndef MATHEMATICS_H
#define MATHEMATICS_H

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2\opencv.hpp>
#include <Eigen/Dense>
#include <vector>

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif


// a 3x3 rotation matrix with R = [[0,1,2],[3,4,5],[6,7,8]]
struct RotM {

	float r[9];

	RotM() {
		for (int i = 0; i < 9; ++i)
			r[i] = 0.0f;
	}

	//Copy rIn to r !! rIn must be a float[9] !!
	RotM(float* rIn) {
		for (int i = 0; i < 9; ++i)
			r[i] = rIn[i];
	}

	/*Calc r with the base vector in the camera system , the result will be r = [[x],[y],[z]]
	ensure, that x,y and z are float[3] !!*/
	RotM(float* x, float* y, float* z) {
		for (int i = 0; i < 3; ++i) {
			r[i] = x[i];
			r[i + 3] = y[i];
			r[i + 6] = z[i];
		}
	}

	RotM(RotM& src) { for (int i = 0; i < 9; ++i) r[i] = src.r[i]; }

	RotM& operator=(RotM& lhs) {
		if (this == &lhs) return *this;
		for (int i = 0; i < 9; ++i) r[i] = lhs.r[i];
		return *this;
	}



};

struct Quaternion {
	double a, b, c, d;

	Quaternion() {

	}

	Quaternion(double _a, double _b, double _c, double _d) {
		a = _a;
		b = _b;
		c = _c;
		d = _d;
	}
};



inline double sq(double a) { return a*a; }

// float input and double input
inline double eucl_Distance(cv::Point2f& p1, cv::Point2f& p2) {
	return sqrt(sq(p1.x - p2.x) + sq(p1.y - p2.y));
}
inline double eucl_Distance(cv::Point2d& p1, cv::Point2d& p2) {
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}
inline double eucl_Distance(cv::Point2f& p1, cv::Point2d& p2) {
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}
inline double eucl_Distance(cv::Point2d& p1, cv::Point2f& p2) {
	return sqrt((p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));
}


// sollte stimmen im vgl mit Anitas Code
inline cv::Mat quaternion2rotM(Quaternion& q) {

	cv::Mat mRotMat = cv::Mat::zeros(cv::Size(3, 3), CV_64FC1);

	// 1. Spalte
	mRotMat.at<cv::Vec2d>(0, 0) = q.d*q.d + q.a*q.a - q.b*q.b - q.c*q.c;
	mRotMat.at<cv::Vec2d>(1, 0) = 2 * (q.a*q.b + q.c*q.d);
	mRotMat.at<cv::Vec2d>(2, 0) = 2 * (q.a*q.c - q.b*q.d);

	// 2.Spalte
	mRotMat.at<cv::Vec2d>(0, 1) = 2 * (q.a*q.b - q.c*q.d);
	mRotMat.at<cv::Vec2d>(1, 1) = q.d*q.d - q.a*q.a + q.b*q.b - q.c*q.c;
	mRotMat.at<cv::Vec2d>(2, 1) = 2 * (q.b*q.c + q.a*q.d);

	// 3. Spalte
	mRotMat.at<cv::Vec2d>(0, 2) = 2 * (q.a*q.c + q.b*q.d);
	mRotMat.at<cv::Vec2d>(1, 2) = 2 * (q.b*q.c - q.a*q.d);
	mRotMat.at<cv::Vec2d>(2, 2) = q.d*q.d - q.a*q.a - q.b*q.b + q.c*q.c;

	std::cout << "Berechne aus Rotationsmatrix: " << mRotMat << std::endl;

	return mRotMat;
}

// read rotation matrix row-major, convert to quaternions
inline Quaternion rotM2quaternion(cv::Mat& rotationsMatrix) {

	Quaternion q;

	if (rotationsMatrix.empty()) {
		std::cout << "no rotationMat available, return!";
		return q;
	}


	double a, b, c, d;
	double r11 = 0, r12 = 0, r13 = 0, r21 = 0, r22 = 0, r23 = 0, r31 = 0, r32 = 0, r33 = 0;

	r11 = rotationsMatrix.at<double>(0, 0);
	r12 = rotationsMatrix.at<double>(0, 1);
	r13 = rotationsMatrix.at<double>(0, 2);

	r21 = rotationsMatrix.at<double>(1, 0);
	r22 = rotationsMatrix.at<double>(1, 1);
	r23 = rotationsMatrix.at<double>(1, 2);

	r31 = rotationsMatrix.at<double>(2, 0);
	r32 = rotationsMatrix.at<double>(2, 1);
	r33 = rotationsMatrix.at<double>(2, 2);


	// TODO schauen wie man sich um die negative wurzel drückt...
	double c_neu = sqrt(((r23 + r32)*(r13 + r31)) / (r12 + r21)) / 2;
	double b_neu = (r12 + r21) / (r13 + r31) * c_neu;
	double a_neu = (r12 + r21) / (4 * b_neu);
	double d_neu = sqrt(1 - a_neu * a_neu - b_neu * b_neu - c_neu*c_neu);

	
	q.a = a_neu;
	q.b = b_neu;
	q.c = c_neu;
	q.d = d_neu;

	//std::cout << "Calculated quaternion: " << q.a << ", b: " << q.b << ", c: " << q.c << ", d: " << q.d << std::endl;

	return q;

}

// calculates the cross product between a and b, returns the solution in c
inline void cross(float* a, float* b, float* c){
c[0] = a[1]*b[2] - a[2]*b[1];
c[1] = a[2]*b[0] - a[0]*b[2];
c[2] = a[0]*b[1] - a[1]*b[0];
}

// calculates the length of the vector in float array t
inline float length(float* t) {
	float r = 0.0f;
	r = t[0] * t[0] + t[1] * t[1] + t[2] * t[2];
	return sqrt(r);
}






// Project points on world´s xy plane 
// rotation matrix column-major
inline std::vector<cv::Point3f> orthoProjection_xyPlaneWorld(std::vector<cv::Point3f>& pointcloud, cv::Point3d& projectionCenter, std::vector<float>& rotationMatrix) {

	std::vector<cv::Point3f> pointsOnXYPlane;
	for (cv::Point3d p : pointcloud) {

		float dx = p.x - projectionCenter.x; //^= X_PC_minus_X0 -> Frank
		float dy = p.y - projectionCenter.y; //^= Y_PC_minus_Y0 -> Frank
		float dz = p.z - projectionCenter.z; //^= Z_PC_minus_Z0 -> Frank

		float x = rotationMatrix[0] * dx + rotationMatrix[3] * dy + rotationMatrix[6] * dz; // ^= x_Kamera -> Frank
		float y = rotationMatrix[1] * dx + rotationMatrix[4] * dy + rotationMatrix[7] * dz; // ^= Y_Kamera -> Frank
		float z = rotationMatrix[2] * dx + rotationMatrix[5] * dy + rotationMatrix[8] * dz; // ^= Z_Kamera -> Frank

		// Orthogonale Projection
		pointsOnXYPlane.push_back(cv::Point3d(x, y, 0));

	}
	return pointsOnXYPlane;
}







inline std::pair<Eigen::Vector3d, Eigen::Vector3d> best_plane_from_points(const std::vector <Eigen::Vector3d> & c) {
	// copy coordinates to  matrix in Eigen format
	size_t num_atoms = c.size();
	Eigen::Matrix< Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic > coord(3, num_atoms);
	for (size_t i = 0; i < num_atoms; ++i) coord.col(i) = c[i];

	// calculate centroid
	Eigen::Vector3d centroid(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

	// subtract centroid
	coord.row(0).array() -= centroid(0); coord.row(1).array() -= centroid(1); coord.row(2).array() -= centroid(2);

	// we only need the left-singular matrix here
	//  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points

	auto svd = coord.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Vector3d plane_normal = svd.matrixU().rightCols<1>();
	//double stabw = svd.matrixU()[0];

	//std::cout << "stabw: " << stabw << std::endl;
	return std::make_pair(centroid, plane_normal);

}






#endif /* MATHEMATICS_H */

