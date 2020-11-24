#ifndef SOLVE_PNP_REIMPLEMENTATION_H_
#define SOLVE_PNP_REIMPLEMENTATION_H_

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
//#include <opencv2/core/internal.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
//#include <opencv2/calib3d/>
#include <opencv2/calib3d/epnp.h>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/calib3d/p3p.h>
//#include <cvconfig.h>


#include "precomp.hpp"
#include "upnp.h"
#include "dls.h"
#include "epnp.h"
#include "p3p.h"
#include "ap3p.h"


#include <opencv2/photo/photo.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/cxcore.h>




#ifdef HAVE_TEGRA_OPTIMIZATION
#include "opencv2/features2d/features2d_tegra.hpp"
#endif

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

inline std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch (depth) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans + '0');

	return r;
}

inline double sq(double a) { return a*a; }

inline bool CompareResponse(const cv::KeyPoint& first, const cv::KeyPoint& second)
{
	if (first.response < second.response)
		return true;
	else
		return false;
}

inline bool CompareQueryID(const cv::DMatch& first, const cv::DMatch& second)
{
	if (first.queryIdx < second.queryIdx)
		return true;
	else
		return false;
}

inline bool CompareTrainID(const cv::DMatch& first, const cv::DMatch& second)
{
	if (first.trainIdx < second.trainIdx)
		return true;
	else
		return false;
}







class PnPRansacCallback : public cv::PointSetRegistrator::Callback
{

public:

	PnPRansacCallback(Mat _cameraMatrix = Mat(3, 3, CV_64F), Mat _distCoeffs = Mat(4, 1, CV_64F), int _flags = SOLVEPNP_ITERATIVE,
		bool _useExtrinsicGuess = false, Mat _rvec = Mat(), Mat _tvec = Mat())
		: cameraMatrix(_cameraMatrix), distCoeffs(_distCoeffs), flags(_flags), useExtrinsicGuess(_useExtrinsicGuess),
		rvec(_rvec), tvec(_tvec) {}

	/* Pre: True */
	/* Post: compute _model with given points and return number of found models */
	int runKernel(InputArray _m1, InputArray _m2, OutputArray _model) const
	{
		Mat opoints = _m1.getMat(), ipoints = _m2.getMat();

		bool correspondence = solvePnP(_m1, _m2, cameraMatrix, distCoeffs,
			rvec, tvec, useExtrinsicGuess, flags);

		Mat _local_model;
		hconcat(rvec, tvec, _local_model);
		_local_model.copyTo(_model);

		return correspondence;
	}

	/* Pre: True */
	/* Post: fill _err with projection errors */
	void computeError(InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err) const
	{

		Mat opoints = _m1.getMat(), ipoints = _m2.getMat(), model = _model.getMat();

		int i, count = opoints.checkVector(3);
		Mat _rvec = model.col(0);
		Mat _tvec = model.col(1);


		Mat projpoints(count, 2, CV_32FC1);
		projectPoints(opoints, _rvec, _tvec, cameraMatrix, distCoeffs, projpoints);

		const Point2f* ipoints_ptr = ipoints.ptr<Point2f>();
		const Point2f* projpoints_ptr = projpoints.ptr<Point2f>();

		_err.create(count, 1, CV_32FC1);
		float* err = _err.getMat().ptr<float>();

		for (i = 0; i < count; ++i)
			err[i] = (float)norm(Matx21f(ipoints_ptr[i] - projpoints_ptr[i]), NORM_L2SQR);

	}


	Mat cameraMatrix;
	Mat distCoeffs;
	int flags;
	bool useExtrinsicGuess;
	Mat rvec;
	Mat tvec;
};

bool solvePnPRansac(InputArray _opoints, InputArray _ipoints,
	InputArray _cameraMatrix, InputArray _distCoeffs,
	OutputArray _rvec, OutputArray _tvec, bool useExtrinsicGuess,
	int iterationsCount, float reprojectionError, double confidence,
	OutputArray _inliers, int flags)
{
	CV_INSTRUMENT_REGION()

		Mat opoints0 = _opoints.getMat(), ipoints0 = _ipoints.getMat();
	Mat opoints, ipoints;
	if (opoints0.depth() == CV_64F || !opoints0.isContinuous())
		opoints0.convertTo(opoints, CV_32F);
	else
		opoints = opoints0;
	if (ipoints0.depth() == CV_64F || !ipoints0.isContinuous())
		ipoints0.convertTo(ipoints, CV_32F);
	else
		ipoints = ipoints0;

	int npoints = std::max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));
	CV_Assert(npoints >= 4 && npoints == std::max(ipoints.checkVector(2, CV_32F), ipoints.checkVector(2, CV_64F)));

	CV_Assert(opoints.isContinuous());
	CV_Assert(opoints.depth() == CV_32F || opoints.depth() == CV_64F);
	CV_Assert((opoints.rows == 1 && opoints.channels() == 3) || opoints.cols*opoints.channels() == 3);
	CV_Assert(ipoints.isContinuous());
	CV_Assert(ipoints.depth() == CV_32F || ipoints.depth() == CV_64F);
	CV_Assert((ipoints.rows == 1 && ipoints.channels() == 2) || ipoints.cols*ipoints.channels() == 2);

	_rvec.create(3, 1, CV_64FC1);
	_tvec.create(3, 1, CV_64FC1);

	Mat rvec = useExtrinsicGuess ? _rvec.getMat() : Mat(3, 1, CV_64FC1);
	Mat tvec = useExtrinsicGuess ? _tvec.getMat() : Mat(3, 1, CV_64FC1);
	Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();

	int model_points = 5;
	int ransac_kernel_method = SOLVEPNP_EPNP;

	if (flags == SOLVEPNP_P3P || flags == SOLVEPNP_AP3P)
	{
		model_points = 4;
		ransac_kernel_method = flags;
	}
	else if (npoints == 4)
	{
		model_points = 4;
		ransac_kernel_method = SOLVEPNP_P3P;
	}

	if (model_points == npoints)
	{
		bool result = solvePnP(opoints, ipoints, cameraMatrix, distCoeffs, _rvec, _tvec, useExtrinsicGuess, ransac_kernel_method);

		if (!result)
		{
			if (_inliers.needed())
				_inliers.release();

			return false;
		}

		if (_inliers.needed())
		{
			_inliers.create(npoints, 1, CV_32S);
			Mat _local_inliers = _inliers.getMat();
			for (int i = 0; i < npoints; i++)
			{
				_local_inliers.at<int>(i) = i;
			}
		}

		return true;
	}

	Ptr<PointSetRegistrator::Callback> cb; // pointer to callback
	cb = makePtr<PnPRansacCallback>(cameraMatrix, distCoeffs, ransac_kernel_method, useExtrinsicGuess, rvec, tvec);

	double param1 = reprojectionError;                // reprojection error
	double param2 = confidence;                       // confidence
	int param3 = iterationsCount;                     // number maximum iterations

	Mat _local_model(3, 2, CV_64FC1);
	Mat _mask_local_inliers(1, opoints.rows, CV_8UC1);

	// call Ransac
	int result = createRANSACPointSetRegistrator(cb, model_points,
		param1, param2, param3)->run(opoints, ipoints, _local_model, _mask_local_inliers);

	if (result <= 0 || _local_model.rows <= 0)
	{
		_rvec.assign(rvec);    // output rotation vector
		_tvec.assign(tvec);    // output translation vector

		if (_inliers.needed())
			_inliers.release();

		return false;
	}

	vector<Point3d> opoints_inliers;
	vector<Point2d> ipoints_inliers;
	opoints = opoints.reshape(3);
	ipoints = ipoints.reshape(2);
	opoints.convertTo(opoints_inliers, CV_64F);
	ipoints.convertTo(ipoints_inliers, CV_64F);

	const uchar* mask = _mask_local_inliers.ptr<uchar>();
	int npoints1 = compressElems(&opoints_inliers[0], mask, 1, npoints);
	compressElems(&ipoints_inliers[0], mask, 1, npoints);

	opoints_inliers.resize(npoints1);
	ipoints_inliers.resize(npoints1);
	result = solvePnP(opoints_inliers, ipoints_inliers, cameraMatrix,
		distCoeffs, rvec, tvec, useExtrinsicGuess,
		(flags == SOLVEPNP_P3P || flags == SOLVEPNP_AP3P) ? SOLVEPNP_EPNP : flags) ? 1 : -1;

	if (result <= 0)
	{
		_rvec.assign(_local_model.col(0));    // output rotation vector
		_tvec.assign(_local_model.col(1));    // output translation vector

		if (_inliers.needed())
			_inliers.release();

		return false;
	}
	else
	{
		_rvec.assign(rvec);    // output rotation vector
		_tvec.assign(tvec);    // output translation vector
	}

	if (_inliers.needed())
	{
		Mat _local_inliers;
		for (int i = 0; i < npoints; ++i)
		{
			if ((int)_mask_local_inliers.at<uchar>(i) != 0) // inliers mask
				_local_inliers.push_back(i);    // output inliers vector
		}
		_local_inliers.copyTo(_inliers);
	}
	return true;
}




























#endif /* SOLVE_PNP_REIMPLEMENTATION_H_ */
