/*
 * AccurateMatcher.h
 *
 *  Created on: Nov 6, 2015
 *      Author: christian
 */

#ifndef ACCURATEMATCHER_H_
#define ACCURATEMATCHER_H_

#include "opencv_includes.h"



class AccurateMatcher {
private:
	// pointer to the feature point detector object
	cv::Ptr<cv::FeatureDetector> detector;
	// pointer to the feature descriptor extractor object
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorExtractor> Oextractor;
	// pointer to the matcher object
	cv::Ptr<cv::DescriptorMatcher > matcher;
	float ratio; // max ratio between 1st and 2nd NN
	bool refineF; // if true will refine the F matrix
	double confidence; // confidence level (probability)
	double distance; // min distance to epipolar
	unsigned int syn_total_points, real_total_points;
	std::vector<int> inliers_syn, inliers_ref;
public:
	AccurateMatcher();
	virtual ~AccurateMatcher();
	// Set the feature detector
	inline void setFeatureDetector(cv::Ptr<cv::FeatureDetector>& detect) { detector= detect; }
	// Set the descriptor extractor
	inline void setDescriptorExtractor(cv::Ptr<cv::DescriptorExtractor>& desc) { extractor= desc; }
	// Set the descriptor extractor
	inline void setDescriptorExtractorOppColor(cv::Ptr<cv::DescriptorExtractor>& desc) { Oextractor= desc; }
	// Set the matcher
	inline void setDescriptorMatcher(cv::Ptr<cv::DescriptorMatcher>& match) { matcher= match; }
	// Set confidence level
	inline void setConfidenceLevel(double conf) { confidence= conf; }
	// Set MinDistanceToEpipolar
	inline void setMinDistanceToEpipolar(double dist) { distance= dist; }
	// Get number of inliers after matching
	inline int getNumberOfInliersSyn() { return inliers_syn.size(); }
	inline int getNumberOfInliersRef() { return inliers_ref.size(); }
	// Get position of inliers after matching
	inline std::vector<int>& getInliersSyn(void) { return inliers_syn; }
	inline std::vector<int>& getInliersRef(void) { return inliers_ref; }

	// Insert symmetrical matches in symMatches vector
	void symmetryTest(const std::vector<cv::DMatch>& matches1, const std::vector<cv::DMatch>& matches2, std::vector<cv::DMatch>& symMatches);

	// Identify good matches using RANSAC
	// Return fundemental matrix
	cv::Mat ransacTest(const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& outMatches);
	// Abwandlung von Christians Matching Test
	static cv::Mat ransacTest_reimpl(std::vector<cv::Point2d>& _points1, std::vector<cv::Point2d>& _points2, bool refineF);
	// Match feature points using symmetry test and RANSAC
	// returns fundemental matrix
	cv::Mat match(cv::Mat& image1, cv::Mat& image2, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2, cv::InputArray& maskROI);
};

#endif /* ACCURATEMATCHER_H_ */


/*
* MSCR parameters

int _mscr_delta = 5; //it compares  (size_{i}-size_{i-delta})/size_{i-delta}. default 5.
int _mscr_min_area = 60; // MinArea prune the area which smaller than minArea. default 60.
int _mscr_max_area = 14400; // MaxArea prune the area which bigger than maxArea. default 14400.
double _mscr_max_variation = 0.25; //MaxVariation prune the area have simliar size to its children. default 0.25
double _mscr_min_diversity = 0.2; //MinDiversity trace back to cut off mser with diversity < min_diversity. default 0.2.
int _mscr_max_evolution = 200; //MaxEvolution for color image, the evolution steps. default 200.
double _mscr_area_threshold = 1.01; //AreaThreshold the area threshold to cause re - initialize. default 1.01.
double _mscr_min_margin = 0.003; //MinMargin ignore too small margin. default 0.003.
int _mscr_edge_blur_size = 5; //EdgeBlurSize the aperture size for edge blur. default 5.
//Mask Optional input mask that marks the regions where we should detect features
*/