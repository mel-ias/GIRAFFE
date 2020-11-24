/*
 * AccurateMatcher.cpp
 *
 *  Created on: Nov 6, 2015
 *      Author: christian
 */

#include "AccurateMatcher.h"

AccurateMatcher::AccurateMatcher() : ratio(1.0), refineF(true), confidence(0.99), distance(3.0), syn_total_points(0), real_total_points(0)
{

	// TODO Auto-generated constructor stub
	// ORB is the default feature
	//detector = cv::ORB::create(); // new cv::OrbFeatureDetector();
	//extractor = cv::ORB::create(); //new cv::OrbDescriptorExtractor();
	//Oextractor = NULL;
	//matcher= new cv::BFMatcher(cv::NORM_HAMMING2, true);
	inliers_ref = std::vector<int>();
	inliers_syn = std::vector<int>();
}

AccurateMatcher::~AccurateMatcher() {
	// TODO Auto-generated destructor stub
}

void AccurateMatcher::symmetryTest(const std::vector<cv::DMatch>& matches1, const std::vector<cv::DMatch>& matches2, std::vector<cv::DMatch>& symMatches)
{
	if(matches1.empty() || matches2.empty())
		return;
    // for all matches image 1 -> image 2
    for (std::vector<cv::DMatch>::const_iterator matchIterator1= matches1.begin();matchIterator1!= matches1.end(); ++matchIterator1) {
    	// for all matches image 2 -> image 1
    	for (std::vector<cv::DMatch>::const_iterator matchIterator2= matches2.begin(); matchIterator2!= matches2.end(); ++matchIterator2) {
    		// Match symmetry test
    		if ( (*matchIterator1).queryIdx == (*matchIterator2).trainIdx && (*matchIterator2).queryIdx ==(*matchIterator1).trainIdx) {
    			// add symmetrical match
    			symMatches.push_back(cv::DMatch((*matchIterator1).queryIdx, (*matchIterator1).trainIdx, (*matchIterator1).distance));
    			break; // next match in image 1 -> image 2
    		}
    	}
    }
}

cv::Mat AccurateMatcher::ransacTest(const std::vector<cv::DMatch>& matches, const std::vector<cv::KeyPoint>& keypoints1, const std::vector<cv::KeyPoint>& keypoints2, std::vector<cv::DMatch>& outMatches)
{
	// Convert keypoints into Point2f
	std::vector<cv::Point2f> points1, points2;
	cv::Mat fundemental;
	for (std::vector<cv::DMatch>:: const_iterator it= matches.begin(); it!= matches.end(); ++it) {
		// Get the position of left keypoints
		float x= keypoints1[it->queryIdx].pt.x;
		float y= keypoints1[it->queryIdx].pt.y;
		points1.push_back(cv::Point2f(x,y));
		// Get the position of right keypoints
		x= keypoints2[it->trainIdx].pt.x;
		y= keypoints2[it->trainIdx].pt.y;
		points2.push_back(cv::Point2f(x,y));
	}
	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(),0);
	if (points1.size()>0&&points2.size()>0){
		cv::Mat fundemental= cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2), // matching points
				inliers,       // match status (inlier or outlier)
				cv::FM_RANSAC, // RANSAC method
				distance,      // distance to epipolar line
				confidence); // confidence probability
		// extract the surviving (inliers) matches
		std::vector<uchar>::const_iterator itIn= inliers.begin();
		std::vector<cv::DMatch>::const_iterator itM= matches.begin();
		// for all matches
		for ( ;itIn!= inliers.end(); ++itIn, ++itM) {
			if (*itIn) { // it is a valid match
				outMatches.push_back(*itM);
			}
		}

		if (refineF) {
			// The F matrix will be recomputed with
			// all accepted matches
			// Convert keypoints into Point2f
			// for final F computation
			points1.clear();
			points2.clear();
			for (std::vector<cv::DMatch>::const_iterator it= outMatches.begin(); it!= outMatches.end(); ++it) {
				// Get the position of left keypoints
				float x= keypoints1[it->queryIdx].pt.x;
				float y= keypoints1[it->queryIdx].pt.y;
				points1.push_back(cv::Point2f(x,y));
				// Get the position of right keypoints
				x= keypoints2[it->trainIdx].pt.x;
				y= keypoints2[it->trainIdx].pt.y;
				points2.push_back(cv::Point2f(x,y));
			}
			// Compute 8-point F from all accepted matches
			if (points1.size()>7 && points2.size()>7){
				fundemental= cv::findFundamentalMat(
						cv::Mat(points1),cv::Mat(points2), // matches
						cv::FM_8POINT); // 8-point method
			}

			if(points1.size()>3 && points2.size()>3)
			{
#ifdef DEBUG
				std::cout << "Num outMatches: " << outMatches.size() << ", Distances: ";
#endif
				cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), cv::LMEDS, distance);
				for(unsigned i = 0; i < outMatches.size(); i++) {
					cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
					col.at<double>(0) = keypoints1[outMatches[i].queryIdx].pt.x;
					col.at<double>(1) = keypoints1[outMatches[i].queryIdx].pt.y;

					col = homography * col;
					col /= col.at<double>(2);
					double dist = sqrt( pow(col.at<double>(0) - keypoints2[outMatches[i].trainIdx].pt.x, 2) +
										pow(col.at<double>(1) - keypoints2[outMatches[i].trainIdx].pt.y, 2));
#ifdef DEBUG
					std::cout << dist << " ";
#endif

					if(dist < distance) {
						inliers_syn.push_back(outMatches[i].queryIdx);
						inliers_ref.push_back(outMatches[i].trainIdx);
					}
				}
			}
		}


#ifdef DEBUG
	    std::cout << std::endl;
#endif
	}
	return fundemental;
}



// Abwandlung von Christians Matching Test
cv::Mat AccurateMatcher::ransacTest_reimpl(std::vector<cv::Point2d>& _points1, std::vector<cv::Point2d>& _points2, bool refineF)
{
	// Convert keypoints into Point2f
	std::vector<cv::Point2d> points1(_points1);
	std::vector<cv::Point2d> points2(_points2);

	// Punkte die Test erfüllen
	std::vector<cv::Point2d> points1_good;
	std::vector<cv::Point2d> points2_good;

	std::vector<cv::Point2d> points1_passed;
	std::vector<cv::Point2d> points2_passed;

	std::vector<int> common_inliers;
	std::vector <int> indices_inliers_fund;
	std::vector <int> indices_inliers_homo;

	cv::Mat fundemental;

	double confidence = 0.95;
	double distance = 5.0;

	// Compute F matrix using RANSAC
	std::vector<uchar> inliers(points1.size(), 0);

	if (points1.size()>0 && points2.size()>0) {

		cv::Mat inliers_fund;
		cv::Mat Fund = cv::findFundamentalMat(
			cv::Mat(points1), cv::Mat(points2),  // matching points
			inliers_fund, // match status (inlier or outlier)
			cv::FM_RANSAC, // RANSAC method
			distance, // distance to epipolar line
			confidence); // confidence probability
						 // extract the surviving (inliers) matches


		std::cout << "RansacTest, Fund = " << std::endl << " " << Fund << std::endl << std::endl;

		// for all matches
		for (int r = 0; r < inliers_fund.rows; r++)
		{
			const uchar* value = inliers_fund.ptr<uchar>(r); // it is a valid match
			if (*value != 0)
				indices_inliers_fund.push_back(r);
		}

		std::cout << "RansacTest, Vsfm, Number of inliers Fundamental Mat, ideal camera (cxr): " << indices_inliers_fund.size() << std::endl;

		// prüfe glelichen Elemente in beiden Vektoren ab
		for (int i = 0; i < _points1.size(); i++)
		{
			if (std::find(indices_inliers_fund.begin(), indices_inliers_fund.end(), i) != indices_inliers_fund.end()) {
				points1_good.push_back(points1[i]);
				points2_good.push_back(points2[i]);
			}
		}

		std::cout << "RansacTest, Size After FundMat: " << points1_good.size() << ", 2: " << points2_good.size() << std::endl;

		if (refineF) {
			// The F matrix will be recomputed with
			// all accepted matches
			// Convert keypoints into Point2f
			// for final F computation
			points1.clear();
			points2.clear();

			// übergebe Punkte die ersten Test FundamentalMat bestanden haben
			points1 = (points1_good);
			points2 = (points2_good);

			// Compute 8-point F from all accepted matches
			if (points1.size()>7 && points2.size()>7) {
				fundemental = cv::findFundamentalMat(
					cv::Mat(points1), cv::Mat(points2), // matches
					cv::FM_8POINT); // 8-point method
			}

			if (points1.size()>3 && points2.size()>3)
			{
#ifdef DEBUG
				std::cout << "Num outMatches: " << outMatches.size() << ", Distances: ";
#endif
				// berechne nun homography 
				cv::Mat homography = cv::findHomography(cv::Mat(points1), cv::Mat(points2), cv::LMEDS, distance);

				for (unsigned i = 0; i < points1_good.size(); i++) {
					cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
					col.at<double>(0) = points1_good[i].x;
					col.at<double>(1) = points1_good[i].y;


					col = homography * col;
					col /= col.at<double>(2);
					double dist = sqrt(pow(col.at<double>(0) - points2_good[i].x, 2) +
						pow(col.at<double>(1) - points2_good[i].y, 2));
#ifdef DEBUG
					std::cout << dist << " ";
#endif

					if (dist < distance) {
						points1_passed.push_back(points1_good[i]);
						points2_passed.push_back(points2_good[i]);
					}
				}
			}
		}


#ifdef DEBUG
		std::cout << std::endl;
#endif
	}

	_points1.clear();
	_points2.clear();

	_points1 = points1_passed;
	_points2 = points2_passed;

	std::cout << "RansacTest, Size of Passed points 1: " << points1_passed.size() << ", 2: " << points2_passed.size() << std::endl;
	return fundemental;
}





cv::Mat AccurateMatcher::match(cv::Mat& image1, cv::Mat& image2, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2, cv::InputArray& maskROI)
{
	// 1a. Detection of the SURF features
#ifdef DEBUG
	std::cout << "Feature Detection ... " << std::endl;
#endif
	detector->detect(image1,keypoints1);
	detector->detect(image2,keypoints2); // maske nur für realImage hier anwenden! 
	if( keypoints1.empty() || keypoints2.empty() )
		return cv::Mat();

#ifdef DEBUG
	std::cout << "Feature Description ... " << std::endl;
#endif
	// 1b. Extraction of the descriptors
	cv::Mat descriptors1, descriptors2;
//	if(Oextractor==NULL)
//	{
		extractor->compute(image1,keypoints1,descriptors1);
#ifdef DEBUG
		std::cout << "FD syn done ... " << std::endl;
#endif
		extractor->compute(image2,keypoints2,descriptors2);
#ifdef DEBUG
		std::cout << "FD real done ... " << std::endl;
#endif
/*	}
	else
	{
		Oextractor->compute(image1,keypoints1,descriptors1);
#ifdef DEBUG
		std::cout << "FD syn done ... " << std::endl;
#endif
		Oextractor->compute(image2,keypoints2,descriptors2);
#ifdef DEBUG
		std::cout << "FD real done ... " << std::endl;
#endif
	}*/
#ifdef DEBUG
	std::cout << "vector sizes: img1 - " << descriptors1.rows << "x" <<descriptors1.cols << ", img2 - " << descriptors2.rows << "x" << descriptors2.cols << ", type: " << type2str(descriptors1.type()) << " ... " << std::endl;
#endif
	if( (descriptors1.rows == 0) || (descriptors1.cols == 0) || (descriptors2.rows == 0) || (descriptors2.cols == 0) )
		return cv::Mat();
	if( (descriptors1.rows < 2) || (descriptors2.rows < 2) )
		return cv::Mat();

	// 2. Match the two image descriptors
	// Construction of the matcher
	//cv::BruteForceMatcher<cv::L2<float>> matcher;
#ifdef DEBUG
	std::cout << "Matching features img1->img2 using distance matching ..." << std::endl;
#endif
	// from image 1 to image 2
	// based on k nearest neighbours (with k=2)
	std::vector<cv::DMatch> matches1;
	try {
		matcher->match(descriptors1, descriptors2, matches1);
	}
	catch(cv::Exception& e) {
		throw cv::Exception(e.code, e.err, "cv::Mat AccurateMatcher::match(cv::Mat& image1, cv::Mat& image2, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2)", __FILE__, __LINE__);
		std::cout << "Error in '"<< e.file <<"' line " << e.line << ", message: " << e.msg << std::endl;
	}

#ifdef DEBUG
	std::cout << "Matching features img1->img2 done." << std::endl;
#endif
	if(matches1.empty())
		return cv::Mat();

#ifdef DEBUG
	std::cout << "Matching features img2->img1 using distance matching..." << std::endl;
#endif
	// from image 2 to image 1
	// based on k nearest neighbours (with k=2)
	std::vector<cv::DMatch> matches2;
	try {
		matcher->match(descriptors2, descriptors1, matches2);
	}
	catch(cv::Exception& e) {
		throw cv::Exception(e.code, e.err, "cv::Mat AccurateMatcher::match(cv::Mat& image1, cv::Mat& image2, std::vector<cv::DMatch>& matches, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2)", __FILE__, __LINE__);
		std::cout << "Error in '"<< e.file <<"' line " << e.line << ", message: " << e.msg << std::endl;
	}

#ifdef DEBUG
	std::cout << "Matching features img2->img1 done." << std::endl;
#endif
	if(matches2.empty())
		return cv::Mat();

#ifdef DEBUG
	std::cout << "Symmetry-test ... " << std::endl;
#endif
	// 4. Remove non-symmetrical matches
	std::vector<cv::DMatch> symMatches;
	symmetryTest(matches1,matches2,symMatches);

#ifdef DEBUG
	std::cout << "Homography-test ... " << std::endl;
#endif
	// 5. Validate matches using RANSAC
	cv::Mat fundemental= ransacTest(symMatches,keypoints1, keypoints2, matches);
	// return the found fundemental matrix
	return fundemental;
}

