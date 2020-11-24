
#ifndef ASIFTDETECTOR_H
#define ASIFTDETECTOR_H

#include <vector>
#include "opencv_includes.h"

using namespace cv;

class ASiftDetector
{
public:
	ASiftDetector();

	void detectAndCompute(const Mat& img, std::vector< KeyPoint >& keypoints, Mat& descriptors);

private:
	void affineSkew(double tilt, double phi, Mat& img, Mat& mask, Mat& Ai);
}; 

# endif
