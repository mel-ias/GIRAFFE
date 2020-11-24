#ifndef OPENCV_INCLUDES_H_
#define OPENCV_INCLUDES_H_

#include "stdafx.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <iomanip>



#include <opencv2\world.hpp>
#include <opencv2\opencv.hpp>
#include <opencv2\flann\miniflann.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <opencv2\xfeatures2d\nonfree.hpp>
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\ml\ml.hpp>
#include <opencv2\photo\photo.hpp>
#include <opencv2\video\video.hpp>
#include <opencv2\highgui\highgui.hpp>

#include <vector>


#include <opencv2\imgproc\imgproc_c.h>
#include <opencv2\core\types_c.h>
#include <opencv2\core\core_c.h>
#include <opencv2\calib3d\calib3d_c.h>
#include <opencv2\highgui\highgui_c.h>
//#include <opencv2\imgcodecs\imgcodecs_c.h>
#include <opencv2\ccalib.hpp>

#include "Mathematics.h"


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



inline void drawMatches(const cv::Mat& img1, const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints2, const std::vector<cv::DMatch>& matches1to2, cv::Mat& outImg, const cv::Scalar& matchColor, const cv::Scalar& otherColor, int ptRadius, int ptThickness, int lineSize, bool showAllPoints = true)
{
	cv::Size sz1 = img1.size();
	cv::Size sz2 = img2.size();
	cv::Mat(sz1.height, sz1.width + sz2.width, CV_8UC3).copyTo(outImg);
	cv::Mat left(outImg, cv::Rect(0, 0, sz1.width, sz1.height));
	img1.copyTo(left);
	cv::Mat right(outImg, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
	img2.copyTo(right);

	cv::Point2f tPoint;
	if (showAllPoints)
	{
		//im3.adjustROI(0, 0, -sz1.width, sz2.width);
		for (unsigned int i = 0; i < keypoints1.size(); ++i)
		{
			cv::circle(outImg, keypoints1.at(i).pt, ptRadius, otherColor, ptThickness, cv::LINE_AA, 0);
		}
		//im3.adjustROI(0, 0, sz1.width, 0);
		for (unsigned int i = 0; i < keypoints2.size(); ++i)
		{
			tPoint = keypoints2.at(i).pt;
			tPoint.x += float(sz1.width);
			cv::circle(outImg, tPoint, ptRadius, otherColor, ptThickness, cv::LINE_AA, 0);
		}
	}

	cv::Point2f tOP;
	for (unsigned int i = 0; i < matches1to2.size(); ++i)
	{
		tOP = keypoints1.at(matches1to2.at(i).queryIdx).pt;
		tPoint = keypoints2.at(matches1to2.at(i).trainIdx).pt;
		tPoint.x += float(sz1.width);
		cv::circle(outImg, tOP, ptRadius, matchColor, -1, cv::LINE_AA, 0);
		cv::circle(outImg, tPoint, ptRadius, matchColor, -1, cv::LINE_AA, 0);
		cv::line(outImg, tOP, tPoint, matchColor, lineSize, cv::LINE_AA, 0);
	}
	
}

inline cv::Vec3b getClampedValue(const cv::Mat& img, long x, long y, long width, long height)
{
	cv::Vec3b result = cv::Vec3b(0, 0, 0);
	if (width == -1)
		width = img.size().width;
	if (height == -1)
		height = img.size().height;
	if ((x > 0) && (x < width) && (y > 0) && (y < height))
		result = img.at<cv::Vec3b>(y, x);
	return result;
}


//http://www.codepool.biz/image-processing-opencv-gamma-correction.html
inline void GammaCorrection(cv::Mat& src, cv::Mat& dst, float fGamma) {

	std::cout << "GammaCorrection" << std::endl;

	unsigned char lut[256];
	for (int i = 0; i < 256; i++) {
		lut[i] = cv::saturate_cast<uchar>(pow((float)(i / 255.0), fGamma) * 255.0f);
	}

	dst = src.clone();
	const int channels = dst.channels();
	switch (channels) {
	case 1: {
		cv::MatIterator_<uchar> it, end;
		for (it = dst.begin<uchar>(), end = dst.end<uchar>(); it != end; it++)
			*it = lut[(*it)];
		break;
	}

	case 3: {
		cv::MatIterator_<cv::Vec3b> it, end;
		for (it = dst.begin<cv::Vec3b>(), end = dst.end<cv::Vec3b>(); it != end; it++) {
			(*it)[0] = lut[((*it)[0])];
			(*it)[1] = lut[((*it)[1])];
			(*it)[2] = lut[((*it)[2])];
		}
		break;
	}
	}
}



inline void WallisGrayscaleChannelOnly(const cv::Mat& imgIn, cv::Mat& outImg, int kernelSize, float targetStDev = 50.0f, float targetMean = 256.0f, float alfa = 0.0f, float limit = 10.0f) {

	std::cout << "WallisGrayscaleChannelOnly" << std::endl;

	cv::Mat ycrcbIn = cv::Mat::zeros(imgIn.size(), CV_8UC3);
	cv::Mat ycrcbOut = cv::Mat::zeros(imgIn.size(), CV_8UC3);
	cv::Mat(imgIn.size().height, imgIn.size().width, CV_8UC3).copyTo(outImg);
	cv::cvtColor(imgIn, ycrcbIn, cv::COLOR_BGR2YCrCb);
	/*
	 * Original Wallis - just on Y-channel
	 */
	cv::Vec3b pix, sout;
	long half = (kernelSize - 1) / 2;
	long w = imgIn.size().width, h = imgIn.size().height;
	float mY, stdY;
	long c, r;
	long xt, yt;
	//int cg = 1; // Wallis standard value
	//float b = 1.5; // Wallis standard value
	//float r1, r0; // Wallis shift and scale parameters
	int size = kernelSize*kernelSize;
	long c_start, c_end, r_start, r_end;
	for (long x = 0; x < w; x++) {
		for (long y = 0; y < h; y++) {
			// compute statistics
			mY = 0;
			c_start = x - half; r_start = y - half;
			c_end = x + half; r_end = y + half;
			for (c = c_start; c < c_end; c++) {
				for (r = r_start; r < r_end; r++) {
					pix = getClampedValue(ycrcbIn, c, r, w, h);
					mY += pix.val[0];
				}
			}
			mY = mY / size;
			stdY = 0;
			for (c = c_start; c < c_end; c++) {
				for (r = r_start; r < r_end; r++) {
					pix = getClampedValue(ycrcbIn, c, r, w, h);
					stdY += sq(pix.val[0] - mY);
				}
			}
			stdY = sqrt(stdY / size);

			//Calc new values
			xt = x; yt = y;
			pix = ycrcbIn.at<cv::Vec3b>(yt, xt);

			//r1 = cg * to_dev / (cg * stdB + to_dev / cg);
			//r0 = b * to_av + (1 - b - r1) * mB;
			//sout.val[0] = pixB.val[0] * r1 + r0 ;
			// HIPS implementation
			if (int(targetMean) == 256)
				sout.val[0] = alfa * pix.val[0] + (1 - alfa) * mY + (pix.val[0] - mY) * targetStDev / (targetStDev / limit + stdY);
			else
				sout.val[0] = alfa * targetMean + (1 - alfa) * mY + (pix.val[0] - mY) * targetStDev / (targetStDev / limit + stdY);

			sout.val[1] = pix.val[1];
			sout.val[2] = pix.val[2];
			// Write new output value
			ycrcbOut.at<cv::Vec3b>(y, x) = sout;
		}
	}

	cv::cvtColor(ycrcbOut, outImg, cv::COLOR_YCrCb2BGR);
	ycrcbIn.release();
	ycrcbOut.release();
}

inline void Wallis(const cv::Mat& imgIn, cv::Mat& outImg, int kernelSize, float targetStDev = 50.0f, float targetMean = 256.0f, float alfa = 0.0f, float limit = 10.0f) {

	std::cout << "Wallis" << std::endl;

	cv::Mat(imgIn.size().height, imgIn.size().width, CV_8UC3).copyTo(outImg);
	//CvScalar pixR, pixB, pixG, sout;
	cv::Vec3b pix, sout;

	long half = (kernelSize - 1) / 2;
	//long x_end = imgIn.size().width - kernelSize;
	//long y_end = imgIn.size().height - kernelSize;
	long w = imgIn.size().width, h = imgIn.size().height;
	float mR, mG, mB, stdR, stdG, stdB;
	long c, r;
	long xt, yt;
	//int cg = 1; // Wallis standard value
	//float b = 1.5; // Wallis standard value
	//float r1, r0; // Wallis shift and scale parameters
	int size = kernelSize*kernelSize;
	long c_start, c_end, r_start, r_end;
	//for (long x = 0; x < x_end; x++){
	//  for (long y = 0; y < y_end; y++){
	for (long x = 0; x < w; x++) {
		for (long y = 0; y < h; y++) {
			// compute statistics
			mR = mG = mB = 0;
			c_start = x - half; r_start = y - half;
			c_end = x + half; r_end = y + half;
			//c_end = x + (kernelSize - 1); r_end = y + kernelSize - 1;
			//for (c = x; c < c_end; c++){
			//  for (r = y; r < r_end; r++){
			for (c = c_start; c < c_end; c++) {
				for (r = r_start; r < r_end; r++) {
					//imgIn.at<cv::Vec3b>(r,c).val[0];
					//pix = imgIn.at<cv::Vec3b>(r,c);
					pix = getClampedValue(imgIn, c, r, w, h);
					mR += pix.val[2];
					mG += pix.val[1];
					mB += pix.val[0];
				}
			}
			mR = mR / size;
			mG = mG / size;
			mB = mB / size;
			stdR = stdB = stdG = 0;
			//for (c = x; c < c_end; c++){
			//  for (r = y; r < r_end; r++){
			for (c = c_start; c < c_end; c++) {
				for (r = r_start; r < r_end; r++) {
					//pix = imgIn.at<cv::Vec3b>(r,c);
					pix = getClampedValue(imgIn, c, r, w, h);
					stdR += sq(pix.val[2] - mR);
					stdG += sq(pix.val[1] - mG);
					stdB += sq(pix.val[0] - mB);
				}
			}
			stdB = sqrt(stdB / size);
			stdR = sqrt(stdR / size);
			stdG = sqrt(stdG / size);

			//Calc new values
			//xt = x+half+1; yt = y+half+1;
			//xt = x+half; yt = y+half;
			xt = x; yt = y;
			pix = imgIn.at<cv::Vec3b>(yt, xt);

			//r1 = cg * to_dev / (cg * stdB + to_dev / cg);
			//r0 = b * to_av + (1 - b - r1) * mB;
			//sout.val[0] = pixB.val[0] * r1 + r0 ;
			// HIPS implementation
			if (int(targetMean) == 256)
				sout.val[0] = alfa * pix.val[0] + (1 - alfa) * mB + (pix.val[0] - mB) * targetStDev / (targetStDev / limit + stdB);
			else
				sout.val[0] = alfa * targetMean + (1 - alfa) * mB + (pix.val[0] - mB) * targetStDev / (targetStDev / limit + stdB);


			//r1 = cg * to_dev / (cg * stdG + to_dev / cg);
			//r0 = b * to_av + (1 - b - r1) * mG;
			//sout.val[1] = pixG.val[0] * r1 + r0 ;
			// HIPS implementation
			if (int(targetMean) == 256)
				sout.val[1] = alfa * pix.val[1] + (1 - alfa) * mG + (pix.val[1] - mG) * targetStDev / (targetStDev / limit + stdG);
			else
				sout.val[1] = alfa * targetMean + (1 - alfa) * mG + (pix.val[1] - mG) * targetStDev / (targetStDev / limit + stdG);

			//r1 = cg * to_dev / (cg * stdR + to_dev / cg);
			//r0 = b * to_av + (1 - b - r1) * mR;
			//sout.val[2] = pixR.val[0] * r1 + r0 ;
			// HIPS implementation
			if (int(targetMean) == 256)
				sout.val[2] = alfa * pix.val[2] + (1 - alfa) * mR + (pix.val[2] - mR) * targetStDev / (targetStDev / limit + stdR);
			else
				sout.val[2] = alfa * targetMean + (1 - alfa) * mR + (pix.val[2] - mR) * targetStDev / (targetStDev / limit + stdR);

			// Write new output value
			//cvSet2D(out, y+half+1, x+half+1, sout);
			//outImg.at<cv::Vec3b>(y+half+1, x+half+1) = sout;
			outImg.at<cv::Vec3b>(y, x) = sout;
		}
#ifdef DEBUG
		fprintf(stderr, "\b\b\b\b\b\b\b\b\b%4ld\\%4ld", x + 1, w);
#endif /* DEBUG */
	}
}

/*

inline void LevMar_Reimplementation(const CvMat* objectPoints, const CvMat* imagePoints, const CvMat* CamMatrixInp, const CvMat* distCoeffs, CvMat* rvec, CvMat* tvec, int useExtrinsicGuess)
{
	std::cout << "LevMar_Reimplementation" << std::endl;

	const int max_iter = 300; //definiere hier iterationen
	cv::Ptr<CvMat> homogCoordObjPkte, _Mxy, _homogCoordBildPkte, _homogCoordBildPktUndistorted, matL;

	int i, count;
	double _data_camMatrix[9], data_camMatrix_new[9] = { 1,0,0,0,1,0,0,0,1 }, R[9];
	double MM[9], U[9], V[9], W[3];
	CvScalar average_HomogCoordObjPkte;
	double param[6]; // Werte von rvec und tvec! --> double[6] = r0 r1 r2 t0 t1 t2 // RotVektor und TranslationsVektor
	CvMat _camMatrix = cvMat(3, 3, CV_64F, _data_camMatrix);
	CvMat _camMatrix_new = cvMat(3, 3, CV_64F, data_camMatrix_new);
	CvMat matR = cvMat(3, 3, CV_64F, R);
	CvMat _r = cvMat(3, 1, CV_64F, param);
	CvMat _t = cvMat(3, 1, CV_64F, param + 3);
	CvMat _matAverage_HomogCoordObjPkt = cvMat(1, 3, CV_64F, average_HomogCoordObjPkte.val); // Matrix zur Verwaltung 
	CvMat _MM = cvMat(3, 3, CV_64F, MM);
	CvMat matU = cvMat(3, 3, CV_64F, U);
	CvMat matV = cvMat(3, 3, CV_64F, V);
	CvMat matW = cvMat(3, 1, CV_64F, W);
	CvMat _param = cvMat(6, 1, CV_64F, param);
	CvMat _dpdr, _dpdt;

	CV_Assert(CV_IS_MAT(objectPoints) && CV_IS_MAT(imagePoints) &&
		CV_IS_MAT(CamMatrixInp) && CV_IS_MAT(rvec) && CV_IS_MAT(tvec));

	count = MAX(objectPoints->cols, objectPoints->rows); //legt zeilenanzahl für cvreshape fest, entweder zeilen oder spalten anzahl
	homogCoordObjPkte = cvCreateMat(1, count, CV_64FC3);
	_homogCoordBildPkte = cvCreateMat(1, count, CV_64FC2);

	cvConvertPointsHomogeneous(objectPoints, homogCoordObjPkte); // homogenen Punktkoordinaten von Objektpunkten
	cvConvertPointsHomogeneous(imagePoints, _homogCoordBildPkte); // homogene Punktkoordinaten von Bildpunkten
	cvConvert(CamMatrixInp, &_camMatrix); // &matA = InputKameraMatrix, konvertiere in matA

	CV_Assert((CV_MAT_DEPTH(rvec->type) == CV_64F || CV_MAT_DEPTH(rvec->type) == CV_32F) &&
		(rvec->rows == 1 || rvec->cols == 1) && rvec->rows*rvec->cols*CV_MAT_CN(rvec->type) == 3);

	CV_Assert((CV_MAT_DEPTH(tvec->type) == CV_64F || CV_MAT_DEPTH(tvec->type) == CV_32F) &&
		(tvec->rows == 1 || tvec->cols == 1) && tvec->rows*tvec->cols*CV_MAT_CN(tvec->type) == 3);

	_homogCoordBildPktUndistorted = cvCreateMat(1, count, CV_64FC2);
	_Mxy = cvCreateMat(1, count, CV_64FC2);

	// normalize image points --> mache Bildpunkte und Verzeichnugsberechnung unabhängig von KameraMatrix
	// (unapply the intrinsic matrix transformation and distortion)
	// camMatrix_new an dieser Stelle Identitätsmatrix --> macht gar nix

	cvUndistortPoints(_homogCoordBildPkte, _homogCoordBildPktUndistorted, &_camMatrix, distCoeffs, 0, &_camMatrix_new);

	cv::Mat plot(_camMatrix_new.rows, _camMatrix_new.cols, CV_64FC1, data_camMatrix_new);
	std::cout << "Spielewiese in LevMarq, plot _camMatrix_new: (muesste I sein): " << plot << std::endl;


	if (useExtrinsicGuess) // wenn näherungen für äußere Orientierung vorhanden
	{
		CvMat _r_temp = cvMat(rvec->rows, rvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
		CvMat _t_temp = cvMat(tvec->rows, tvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);
		cvConvert(rvec, &_r_temp); // konvertiere rvec in r_temp
		cvConvert(tvec, &_t_temp); // konvertiere tvec in t_temp
	}
	else
	{
		average_HomogCoordObjPkte = cvAvg(homogCoordObjPkte); // average_HomogCoordObjPkte = Skalarwert[3], berechne Mittelwert aller Punkte in in homogCoordObjPkte
		cvReshape(homogCoordObjPkte, homogCoordObjPkte, 1, count); //Ändert Form von Matrix ohne Daten zu kopieren, matM = InputArray, matM = Header von Matrix die gefüllt werden soll, neue Anzahl an Kanälen, neue Zeilenanzahl

		//  dst=(src-delta)^T*(src-delta)
		cvMulTransposed(homogCoordObjPkte, &_MM, 1, &_matAverage_HomogCoordObjPkt); // berechnet &_MM = (matM - &_Mc)^T * (matM - &_Mc)

		// berechnet SVD (SingulärwertZerlegung) 
		cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);

		// initialize extrinsic parameters
		if (W[2] / W[1] < 1e-3 || count < 4)
		{
			// a planar structure case (all M's// Objektpunkte lie in the same plane)
			double tt[3], h[9], h1_norm, h2_norm;
			CvMat* R_transform = &matV;
			CvMat T_transform = cvMat(3, 1, CV_64F, tt);
			CvMat matH = cvMat(3, 3, CV_64F, h);
			CvMat _h1, _h2, _h3;

			if (V[2] * V[2] + V[5] * V[5] < 1e-10)
				cvSetIdentity(R_transform);

			if (cvDet(R_transform) < 0)
				cvScale(R_transform, R_transform, -1);

			cvGEMM(R_transform, &_matAverage_HomogCoordObjPkt, -1, 0, 0, &T_transform, CV_GEMM_B_T);

			for (i = 0; i < count; i++)
			{
				const double* Rp = R_transform->data.db;
				const double* Tp = T_transform.data.db;
				const double* src = homogCoordObjPkte->data.db + i * 3;
				double* dst = _Mxy->data.db + i * 2;

				dst[0] = Rp[0] * src[0] + Rp[1] * src[1] + Rp[2] * src[2] + Tp[0];
				dst[1] = Rp[3] * src[0] + Rp[4] * src[1] + Rp[5] * src[2] + Tp[1];
			}

			cvFindHomography(_Mxy, _homogCoordBildPktUndistorted, &matH);

			if (cvCheckArr(&matH, CV_CHECK_QUIET))
			{
				cvGetCol(&matH, &_h1, 0);
				_h2 = _h1; _h2.data.db++;
				_h3 = _h2; _h3.data.db++;
				h1_norm = sqrt(h[0] * h[0] + h[3] * h[3] + h[6] * h[6]);
				h2_norm = sqrt(h[1] * h[1] + h[4] * h[4] + h[7] * h[7]);

				cvScale(&_h1, &_h1, 1. / MAX(h1_norm, DBL_EPSILON));
				cvScale(&_h2, &_h2, 1. / MAX(h2_norm, DBL_EPSILON));
				cvScale(&_h3, &_t, 2. / MAX(h1_norm + h2_norm, DBL_EPSILON));
				cvCrossProduct(&_h1, &_h2, &_h3);

				cvRodrigues2(&matH, &_r);
				cvRodrigues2(&_r, &matH);
				cvMatMulAdd(&matH, &T_transform, &_t, &_t);
				cvMatMul(&matH, R_transform, &matR);
			}
			else
			{
				cvSetIdentity(&matR);
				cvZero(&_t);
			}

			cvRodrigues2(&matR, &_r);
		}
		else
		{
			// non-planar structure. Use DLT method
			double* L; // L = Designmatrix
			double LL[12 * 12], LW[12], LV[12 * 12], sc;
			CvMat _LL = cvMat(12, 12, CV_64F, LL);
			CvMat _LW = cvMat(12, 1, CV_64F, LW);
			CvMat _LV = cvMat(12, 12, CV_64F, LV);
			CvMat _RRt, _RR, _tt;
			CvPoint3D64f* M = (CvPoint3D64f*)homogCoordObjPkte->data.db;
			CvPoint2D64f* mn = (CvPoint2D64f*)_homogCoordBildPktUndistorted->data.db;

			matL = cvCreateMat(2 * count, 12, CV_64F); // matL = DesignMatrix
			L = matL->data.db; // L inhalt von Matrix

			for (i = 0; i < count; i++, L += 24)
			{
				double x = -mn[i].x, y = -mn[i].y;
				L[0] = L[16] = M[i].x;
				L[1] = L[17] = M[i].y;
				L[2] = L[18] = M[i].z;
				L[3] = L[19] = 1.;
				L[4] = L[5] = L[6] = L[7] = 0.;
				L[12] = L[13] = L[14] = L[15] = 0.;
				L[8] = x*M[i].x;
				L[9] = x*M[i].y;
				L[10] = x*M[i].z;
				L[11] = x;
				L[20] = y*M[i].x;
				L[21] = y*M[i].y;
				L[22] = y*M[i].z;
				L[23] = y;
			}

			//std::cout << "Spielewiese in LevMarq, Gesamtgröße Beobachtungsmatrix/Designmatrix L: height: " << matL->height << ", width: " << matL->width << std::endl;


			cvMulTransposed(matL, &_LL, 1);
			cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
			_RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
			cvGetCols(&_RRt, &_RR, 0, 3);
			cvGetCol(&_RRt, &_tt, 3);
			if (cvDet(&_RR) < 0)
				cvScale(&_RRt, &_RRt, -1);
			sc = cvNorm(&_RR);
			cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
			cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
			cvScale(&_tt, &_t, cvNorm(&matR) / sc);
			cvRodrigues2(&matR, &_r);
		}
	}


	// Ende Näherungen Äußere Orientierung

	cvReshape(homogCoordObjPkte, homogCoordObjPkte, 3, 1);
	cvReshape(_homogCoordBildPktUndistorted, _homogCoordBildPktUndistorted, 2, 1);

#ifdef DEBUG
	std::cout << "Max Iterations: " << max_iter << ", Target error rate: " << DBL_EPSILON << std::endl;
#endif

	// refine extrinsic parameters using iterative algorithm
	CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, DBL_EPSILON), true);
	cvCopy(&_param, solver.param);

	unsigned int iter_done = 0; double repr_err = 0;

	for (;;)
	{
		CvMat *matJ = 0, *_err = 0;
		const CvMat *__param = 0;
		bool proceed = solver.update(__param, matJ, _err);
		cvCopy(__param, &_param);
		if (!proceed || !_err)
			break;
		cvReshape(_err, _err, 2, 1);
		if (matJ)
		{
			cvGetCols(matJ, &_dpdr, 0, 3);
			cvGetCols(matJ, &_dpdt, 3, 6);
			cvProjectPoints2(homogCoordObjPkte, &_r, &_t, &_camMatrix, distCoeffs,
				_err, &_dpdr, &_dpdt, 0, 0, 0);
		}
		else
		{
			cvProjectPoints2(homogCoordObjPkte, &_r, &_t, &_camMatrix, distCoeffs,
				_err, 0, 0, 0, 0, 0);
		}
		cvSub(_err, _homogCoordBildPkte, _err);
		cvReshape(_err, _err, 1, 2 * count);

		repr_err = cvAvg(_err).val[0];

		iter_done++;
	}

#ifdef DEBUG
	std::cout << "Iterations done: " << iter_done << ", Error rate: " << repr_err << std::endl;
#endif

	cvCopy(solver.param, &_param);

	_r = cvMat(rvec->rows, rvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
	_t = cvMat(tvec->rows, tvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);

	cvConvert(&_r, rvec);
	cvConvert(&_t, tvec);
}


// nutzt bearbeite Version von LM.
inline bool solvePnP_reimp(cv::InputArray _opoints, cv::InputArray _ipoints, cv::InputArray _cameraMatrix, cv::InputArray _distCoeffs, cv::OutputArray _rvec, cv::OutputArray _tvec, bool useExtrinsicGuess, int flags)
{
	std::cout << "Solve PnP reimpl" << std::endl;

	cv::Mat opoints = _opoints.getMat(), ipoints = _ipoints.getMat();
	int npoints = std::max(opoints.checkVector(3, CV_32F), opoints.checkVector(3, CV_64F));
	CV_Assert(npoints >= 0 && npoints == std::max(ipoints.checkVector(2, CV_32F), ipoints.checkVector(2, CV_64F)));
	_rvec.create(3, 1, CV_64F);
	_tvec.create(3, 1, CV_64F);
	cv::Mat cameraMatrix = _cameraMatrix.getMat(), distCoeffs = _distCoeffs.getMat();

	if (flags == cv::SOLVEPNP_ITERATIVE)
	{
		CvMat c_objectPoints = opoints, c_imagePoints = ipoints;
		CvMat c_cameraMatrix = cameraMatrix, c_distCoeffs = distCoeffs;
		CvMat c_rvec = _rvec.getMat(), c_tvec = _tvec.getMat();

		cv::solvePnPRefineLM(&c_objectPoints, &c_imagePoints, &c_cameraMatrix,
			c_distCoeffs.rows * c_distCoeffs.cols ? &c_distCoeffs : 0,
			&c_rvec, &c_tvec, useExtrinsicGuess);
		
		CvLevMarq();
		LevMar_Reimplementation(&c_objectPoints, &c_imagePoints, &c_cameraMatrix,
			c_distCoeffs.rows*c_distCoeffs.cols ? &c_distCoeffs : 0,
			&c_rvec, &c_tvec, useExtrinsicGuess);

		//cv::Mat camMat(c_cameraMatrix.rows, c_cameraMatrix.cols, CV_64FC1, c_cameraMatrix.data.db);
		//std::cout << "Spielewiese: CameraMatrix LevenbergMargquard :" << camMat << std::endl;
		return true;
	}
	else
		CV_Error(CV_StsBadArg, "The flags argument must be one of CV_ITERATIVE or CV_EPNP");
	return false;
}
*/






inline static void collectCalibrationData_reimpl(cv::InputArrayOfArrays objectPoints,
	cv::InputArrayOfArrays imagePoints1,
	cv::InputArrayOfArrays imagePoints2,
	cv::Mat& objPtMat, cv::Mat& imgPtMat1, cv::Mat* imgPtMat2,
	cv::Mat& npoints)
{
	int nimages = (int)objectPoints.total();
	int i, j = 0, ni = 0, total = 0;
	CV_Assert(nimages > 0 && nimages == (int)imagePoints1.total() &&
		(!imgPtMat2 || nimages == (int)imagePoints2.total()));

	for (i = 0; i < nimages; i++)
	{
		ni = objectPoints.getMat(i).checkVector(3, CV_32F);
		if (ni <= 0)
			CV_Error(CV_StsUnsupportedFormat, "objectPoints should contain vector of vectors of points of type Point3f");
		int ni1 = imagePoints1.getMat(i).checkVector(2, CV_32F);
		if (ni1 <= 0)
			CV_Error(CV_StsUnsupportedFormat, "imagePoints1 should contain vector of vectors of points of type Point2f");
		CV_Assert(ni == ni1);

		total += ni;
	}

	npoints.create(1, (int)nimages, CV_32S);
	objPtMat.create(1, (int)total, CV_32FC3);
	imgPtMat1.create(1, (int)total, CV_32FC2);
	cv::Point2f* imgPtData2 = 0;

	if (imgPtMat2)
	{
		imgPtMat2->create(1, (int)total, CV_32FC2);
		imgPtData2 = imgPtMat2->ptr<cv::Point2f>();
	}

	cv::Point3f* objPtData = objPtMat.ptr<cv::Point3f>();
	cv::Point2f* imgPtData1 = imgPtMat1.ptr<cv::Point2f>();

	for (i = 0; i < nimages; i++, j += ni)
	{
		cv::Mat objpt = objectPoints.getMat(i);
		cv::Mat imgpt1 = imagePoints1.getMat(i);
		ni = objpt.checkVector(3, CV_32F);
		npoints.at<int>(i) = ni;
		memcpy(objPtData + j, objpt.ptr(), ni * sizeof(objPtData[0]));
		memcpy(imgPtData1 + j, imgpt1.ptr(), ni * sizeof(imgPtData1[0]));

		if (imgPtData2)
		{
			cv::Mat imgpt2 = imagePoints2.getMat(i);
			int ni2 = imgpt2.checkVector(2, CV_32F);
			CV_Assert(ni == ni2);
			memcpy(imgPtData2 + j, imgpt2.ptr(), ni * sizeof(imgPtData2[0]));
		}
	}
}


inline static cv::Mat prepareCameraMatrix_reimpl(cv::Mat& cameraMatrix0, int rtype)
{
	cv::Mat cameraMatrix = cv::Mat::eye(3, 3, rtype);
	if (cameraMatrix0.size() == cameraMatrix.size())
		cameraMatrix0.convertTo(cameraMatrix, rtype);
	return cameraMatrix;
}

inline static cv::Mat prepareDistCoeffs_reimpl(cv::Mat& distCoeffs0, int rtype, int outputSize = 14)
{
	CV_Assert((int)distCoeffs0.total() <= outputSize);
	cv::Mat distCoeffs = cv::Mat::zeros(distCoeffs0.cols == 1 ? cv::Size(1, outputSize) : cv::Size(outputSize, 1), rtype);
	if (distCoeffs0.size() == cv::Size(1, 4) ||
		distCoeffs0.size() == cv::Size(1, 5) ||
		distCoeffs0.size() == cv::Size(1, 8) ||
		distCoeffs0.size() == cv::Size(1, 12) ||
		distCoeffs0.size() == cv::Size(1, 14) ||
		distCoeffs0.size() == cv::Size(4, 1) ||
		distCoeffs0.size() == cv::Size(5, 1) ||
		distCoeffs0.size() == cv::Size(8, 1) ||
		distCoeffs0.size() == cv::Size(12, 1) ||
		distCoeffs0.size() == cv::Size(14, 1))
	{
		cv::Mat dstCoeffs(distCoeffs, cv::Rect(0, 0, distCoeffs0.cols, distCoeffs0.rows));
		distCoeffs0.convertTo(dstCoeffs, rtype);
	}
	return distCoeffs;
}


inline void subMatrix_reimpl(const cv::Mat& src, cv::Mat& dst, const std::vector<uchar>& cols,
	const std::vector<uchar>& rows) {
	int nonzeros_cols = cv::countNonZero(cols);
	cv::Mat tmp(src.rows, nonzeros_cols, CV_64FC1);

	for (int i = 0, j = 0; i < (int)cols.size(); i++)
	{
		if (cols[i])
		{
			src.col(i).copyTo(tmp.col(j++));
		}
	}

	int nonzeros_rows = cv::countNonZero(rows);
	dst.create(nonzeros_rows, nonzeros_cols, CV_64FC1);
	for (int i = 0, j = 0; i < (int)rows.size(); i++)
	{
		if (rows[i])
		{
			tmp.row(i).copyTo(dst.row(j++));
		}
	}
}


/*
inline void cvFindExtrinsicCameraParams2_reimpl(const CvMat* objectPoints,
	const CvMat* imagePoints, const CvMat* A,
	const CvMat* distCoeffs, CvMat* rvec, CvMat* tvec,
	int useExtrinsicGuess)
{
	const int max_iter = 300; //Änderung von 20 Iterationen auf 300;
	cv::Ptr<CvMat> matM, _Mxy, _m, _mn, matL;

	int i, count;
	double a[9], ar[9] = { 1,0,0,0,1,0,0,0,1 }, R[9];
	double MM[9], U[9], V[9], W[3];
	CvScalar Mc;
	double param[6];
	CvMat matA = cvMat(3, 3, CV_64F, a);
	CvMat _Ar = cvMat(3, 3, CV_64F, ar);
	CvMat matR = cvMat(3, 3, CV_64F, R);
	CvMat _r = cvMat(3, 1, CV_64F, param);
	CvMat _t = cvMat(3, 1, CV_64F, param + 3);
	CvMat _Mc = cvMat(1, 3, CV_64F, Mc.val);
	CvMat _MM = cvMat(3, 3, CV_64F, MM);
	CvMat matU = cvMat(3, 3, CV_64F, U);
	CvMat matV = cvMat(3, 3, CV_64F, V);
	CvMat matW = cvMat(3, 1, CV_64F, W);
	CvMat _param = cvMat(6, 1, CV_64F, param);
	CvMat _dpdr, _dpdt;

	CV_Assert(CV_IS_MAT(objectPoints) && CV_IS_MAT(imagePoints) &&
		CV_IS_MAT(A) && CV_IS_MAT(rvec) && CV_IS_MAT(tvec));

	count = MAX(objectPoints->cols, objectPoints->rows);
	matM.reset(cvCreateMat(1, count, CV_64FC3));
	_m.reset(cvCreateMat(1, count, CV_64FC2));

	cvConvertPointsHomogeneous(objectPoints, matM);
	cvConvertPointsHomogeneous(imagePoints, _m);
	cvConvert(A, &matA);

	CV_Assert((CV_MAT_DEPTH(rvec->type) == CV_64F || CV_MAT_DEPTH(rvec->type) == CV_32F) &&
		(rvec->rows == 1 || rvec->cols == 1) && rvec->rows*rvec->cols*CV_MAT_CN(rvec->type) == 3);

	CV_Assert((CV_MAT_DEPTH(tvec->type) == CV_64F || CV_MAT_DEPTH(tvec->type) == CV_32F) &&
		(tvec->rows == 1 || tvec->cols == 1) && tvec->rows*tvec->cols*CV_MAT_CN(tvec->type) == 3);

	_mn.reset(cvCreateMat(1, count, CV_64FC2));
	_Mxy.reset(cvCreateMat(1, count, CV_64FC2));

	// normalize image points
	// (unapply the intrinsic matrix transformation and distortion)
	cvUndistortPoints(_m, _mn, &matA, distCoeffs, 0, &_Ar);

	if (useExtrinsicGuess)
	{
		CvMat _r_temp = cvMat(rvec->rows, rvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
		CvMat _t_temp = cvMat(tvec->rows, tvec->cols,
			CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);
		cvConvert(rvec, &_r_temp);
		cvConvert(tvec, &_t_temp);
	}
	else
	{
		Mc = cvAvg(matM);
		cvReshape(matM, matM, 1, count);
		cvMulTransposed(matM, &_MM, 1, &_Mc);
		cvSVD(&_MM, &matW, 0, &matV, CV_SVD_MODIFY_A + CV_SVD_V_T);

		// initialize extrinsic parameters
		if (W[2] / W[1] < 1e-3 || count < 4)
		{
			// a planar structure case (all M's lie in the same plane) // nur für 3D-2D implementiert an dieser Stelle:
			std::cout << "diese OCV Änderung ist nur für 3D-2D Testfeld-Bild Auswertungen optimiert!";
			return;
			double tt[3], h[9], h1_norm, h2_norm;
			CvMat* R_transform = &matV;
			CvMat T_transform = cvMat(3, 1, CV_64F, tt);
			CvMat matH = cvMat(3, 3, CV_64F, h);
			CvMat _h1, _h2, _h3;

			if (V[2] * V[2] + V[5] * V[5] < 1e-10)
				cvSetIdentity(R_transform);

			if (cvDet(R_transform) < 0)
				cvScale(R_transform, R_transform, -1);

			cvGEMM(R_transform, &_Mc, -1, 0, 0, &T_transform, CV_GEMM_B_T);

			for (i = 0; i < count; i++)
			{
				const double* Rp = R_transform->data.db;
				const double* Tp = T_transform.data.db;
				const double* src = matM->data.db + i * 3;
				double* dst = _Mxy->data.db + i * 2;

				dst[0] = Rp[0] * src[0] + Rp[1] * src[1] + Rp[2] * src[2] + Tp[0];
				dst[1] = Rp[3] * src[0] + Rp[4] * src[1] + Rp[5] * src[2] + Tp[1];
			}

			cvFindHomography(_Mxy, _mn, &matH);

			if (cvCheckArr(&matH, CV_CHECK_QUIET))
			{
				cvGetCol(&matH, &_h1, 0);
				_h2 = _h1; _h2.data.db++;
				_h3 = _h2; _h3.data.db++;
				h1_norm = std::sqrt(h[0] * h[0] + h[3] * h[3] + h[6] * h[6]);
				h2_norm = std::sqrt(h[1] * h[1] + h[4] * h[4] + h[7] * h[7]);

				cvScale(&_h1, &_h1, 1. / MAX(h1_norm, DBL_EPSILON));
				cvScale(&_h2, &_h2, 1. / MAX(h2_norm, DBL_EPSILON));
				cvScale(&_h3, &_t, 2. / MAX(h1_norm + h2_norm, DBL_EPSILON));
				cvCrossProduct(&_h1, &_h2, &_h3);

				cvRodrigues2(&matH, &_r);
				cvRodrigues2(&_r, &matH);
				cvMatMulAdd(&matH, &T_transform, &_t, &_t);
				cvMatMul(&matH, R_transform, &matR);
			}
			else
			{
				cvSetIdentity(&matR);
				cvZero(&_t);
			}

			cvRodrigues2(&matR, &_r);
			
		}
		else
		{
			// non-planar structure. Use DLT method
			double* L;
			double LL[12 * 12], LW[12], LV[12 * 12], sc;
			CvMat _LL = cvMat(12, 12, CV_64F, LL);
			CvMat _LW = cvMat(12, 1, CV_64F, LW);
			CvMat _LV = cvMat(12, 12, CV_64F, LV);
			CvMat _RRt, _RR, _tt;
			CvPoint3D64f* M = (CvPoint3D64f*)matM->data.db;
			CvPoint2D64f* mn = (CvPoint2D64f*)_mn->data.db;

			matL.reset(cvCreateMat(2 * count, 12, CV_64F));
			L = matL->data.db;

			for (i = 0; i < count; i++, L += 24)
			{
				double x = -mn[i].x, y = -mn[i].y;
				L[0] = L[16] = M[i].x;
				L[1] = L[17] = M[i].y;
				L[2] = L[18] = M[i].z;
				L[3] = L[19] = 1.;
				L[4] = L[5] = L[6] = L[7] = 0.;
				L[12] = L[13] = L[14] = L[15] = 0.;
				L[8] = x*M[i].x;
				L[9] = x*M[i].y;
				L[10] = x*M[i].z;
				L[11] = x;
				L[20] = y*M[i].x;
				L[21] = y*M[i].y;
				L[22] = y*M[i].z;
				L[23] = y;
			}

			cvMulTransposed(matL, &_LL, 1);
			cvSVD(&_LL, &_LW, 0, &_LV, CV_SVD_MODIFY_A + CV_SVD_V_T);
			_RRt = cvMat(3, 4, CV_64F, LV + 11 * 12);
			cvGetCols(&_RRt, &_RR, 0, 3);
			cvGetCol(&_RRt, &_tt, 3);
			if (cvDet(&_RR) < 0)
				cvScale(&_RRt, &_RRt, -1);
			sc = cvNorm(&_RR);
			cvSVD(&_RR, &matW, &matU, &matV, CV_SVD_MODIFY_A + CV_SVD_U_T + CV_SVD_V_T);
			cvGEMM(&matU, &matV, 1, 0, 0, &matR, CV_GEMM_A_T);
			cvScale(&_tt, &_t, cvNorm(&matR) / sc);
			cvRodrigues2(&matR, &_r);
		}
	}

	cvReshape(matM, matM, 3, 1);
	cvReshape(_mn, _mn, 2, 1);

	// refine extrinsic parameters using iterative algorithm
	CvLevMarq solver(6, count * 2, cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, max_iter, FLT_EPSILON), true);
	cvCopy(&_param, solver.param);

	for (;;)
	{
		CvMat *matJ = 0, *_err = 0;
		const CvMat *__param = 0;
		bool proceed = solver.update(__param, matJ, _err);
		cvCopy(__param, &_param);
		if (!proceed || !_err)
			break;
		cvReshape(_err, _err, 2, 1);
		if (matJ)
		{
			cvGetCols(matJ, &_dpdr, 0, 3);
			cvGetCols(matJ, &_dpdt, 3, 6);
			cvProjectPoints2(matM, &_r, &_t, &matA, distCoeffs,
				_err, &_dpdr, &_dpdt, 0, 0, 0);
		}
		else
		{
			cvProjectPoints2(matM, &_r, &_t, &matA, distCoeffs,
				_err, 0, 0, 0, 0, 0);
		}
		cvSub(_err, _m, _err);
		cvReshape(_err, _err, 1, 2 * count);
	}
	cvCopy(solver.param, &_param);

	_r = cvMat(rvec->rows, rvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(rvec->type)), param);
	_t = cvMat(tvec->rows, tvec->cols,
		CV_MAKETYPE(CV_64F, CV_MAT_CN(tvec->type)), param + 3);

	cvConvert(&_r, rvec);
	cvConvert(&_t, tvec);
}

*/



//output, image (e.g. float) to Textfile!, just one channel!
inline void cv_32FC1_16FC1_image2Txt(const cv::Mat& image, std::string fileName) {

	fileName.append(".txt");
	std::ofstream outStream = std::ofstream(fileName);

	if (outStream.is_open())
	{
		for (int r = 0; r < image.rows; r++)
			for (int c = 0; c < image.cols; c++)
				if (image.ptr<double>(r)[c] != 0)
					outStream << r << "," << c << "," << image.ptr<double>(r)[c] << std::endl;

		outStream.close();
		std::cout << " done saving coord image to " << fileName << std::endl;
	}
	else std::cout << "Unable to open file";
}


// https://stackoverflow.com/a/39113857
inline void sharpen(const cv::Mat& img, cv::Mat& result) {

	std::cout << "Sharpen Image" << std::endl;

	result.create(img.size(), img.type());
	//Processing the inner edge of the pixel point, the image of the outer edge of the pixel should be additional processing
	for (int row = 1; row < img.rows - 1; row++)
	{
		//Front row pixel
		const uchar* previous = img.ptr<const uchar>(row - 1);
		//Current line to be processed
		const uchar* current = img.ptr<const uchar>(row);
		//new row
		const uchar* next = img.ptr<const uchar>(row + 1);
		uchar *output = result.ptr<uchar>(row);
		int ch = img.channels();
		int starts = ch;
		int ends = (img.cols - 1) * ch;
		for (int col = starts; col < ends; col++)
		{
			//The traversing pointer of the output image is synchronized with the current row, and each channel value of each pixel in each row is given a increment, because the channel number of the image is to be taken into account.
			*output++ = cv::saturate_cast<uchar>(5 * current[col] - current[col - ch] - current[col + ch] - previous[col] - next[col]);
		}
	} //end loop
	  //Processing boundary, the peripheral pixel is set to 0
	result.row(0).setTo(cv::Scalar::all(0));
	result.row(result.rows - 1).setTo(cv::Scalar::all(0));
	result.col(0).setTo(cv::Scalar::all(0));
	result.col(result.cols - 1).setTo(cv::Scalar::all(0));
}



//http://docs.opencv.org/2.4/modules/imgproc/doc/histograms.html?highlight=equalizehist
void inline equalizeHistogram(const cv::Mat& img, cv::Mat& result, const cv::Mat& mask = cv::Mat()) {

	std::cout << "Equalize Histogram" << std::endl;

	result = cv::Mat(img.size(), img.type());
	cv::Mat imgB(img.size(), CV_8UC1);
	cv::Mat imgG(img.size(), CV_8UC1);
	cv::Mat imgR(img.size(), CV_8UC1);
	cv::Vec3b pixel;

	for (int r = 0; r < img.rows; r++) {
		for (int c = 0; c < img.cols; c++) {
			pixel = img.at<cv::Vec3b>(r, c);
			imgB.at<uchar>(r, c) = pixel[0];
			imgG.at<uchar>(r, c) = pixel[1];
			imgR.at<uchar>(r, c) = pixel[2];
		}
	}

	equalizeHist(imgB, imgB);
	equalizeHist(imgG, imgG);
	equalizeHist(imgR, imgR);

	for (int r = 0; r < img.rows; r++)
	{
		for (int c = 0; c < img.cols; c++)
		{
			pixel = cv::Vec3b(imgB.at<uchar>(r, c), imgG.at<uchar>(r, c), imgR.at<uchar>(r, c));
			result.at<cv::Vec3b>(r, c) = pixel;
		}
	}
}


//http://docs.opencv.org/2.4/modules/imgproc/doc/histograms.html?highlight=equalizehist
//https://opencv-srf.blogspot.de/2013/08/histogram-equalization.html
void inline equalizeHistogramHSV(const cv::Mat& img, cv::Mat& result, const cv::Mat& mask = cv::Mat()) {

	std::cout << "Equalize Histogram HSV" << std::endl;

	std::vector<cv::Mat> channels;
	cv::Mat img_hist_equalized;

	cv::cvtColor(img, img_hist_equalized, cv::COLOR_BGR2YCrCb); //change the color image from BGR to YCrCb format

	cv::split(img_hist_equalized, channels); //split the image into channels
	cv::equalizeHist(channels[0], channels[0]); //equalize histogram on the 1st channel (Y)
	cv::merge(channels, img_hist_equalized); //merge 3 channels including the modified 1st channel into one image

	cv::cvtColor(img_hist_equalized, img_hist_equalized, cv::COLOR_YCrCb2BGR); //change the color image from YCrCb to BGR format (to display image properly)

	result = img_hist_equalized;



	//cv::Mat hsv, _result;
	//cv::cvtColor(img, hsv, CV_BGR2HSV);

	//_result = cv::Mat(hsv.size(), hsv.type());
	//result = cv::Mat(img.size(), img.type());

	//cv::Mat imgH(hsv.size(), CV_8UC1);
	//cv::Mat imgS(hsv.size(), CV_8UC1);
	//cv::Mat imgV(hsv.size(), CV_8UC1);
	//cv::Vec3b pixel;


	//for (int r = 0; r < hsv.rows; r++) {
	//	for (int c = 0; c < hsv.cols; c++) {
	//		pixel = hsv.at<cv::Vec3b>(r, c);
	//		imgH.at<uchar>(r, c) = pixel[0];
	//		imgS.at<uchar>(r, c) = pixel[1];
	//		imgV.at<uchar>(r, c) = pixel[2];
	//	}
	//}

	////equalizeHist(imgH, imgH);
	//equalizeHist(imgS, imgS);
	////equalizeHist(imgV, imgV);

	//for (int r = 0; r < hsv.rows; r++) {
	//	for (int c = 0; c < hsv.cols; c++) {
	//		pixel = cv::Vec3b(imgH.at<uchar>(r, c), imgS.at<uchar>(r, c), imgV.at<uchar>(r, c));
	//		_result.at<cv::Vec3b>(r, c) = pixel;
	//	}
	//}

	//cv::cvtColor(_result, result, CV_HSV2BGR);

	////cv::cvtColor(hsv, result, CV_HSV2BGR);

	//cv::namedWindow("Window", CV_WINDOW_KEEPRATIO);
	//cv::namedWindow("Window", CV_WINDOW_NORMAL);
	//cv::imshow("Window", result);
	//cv::waitKey();
}





typedef cv::KeyPoint Feature;
//typedef cv::Mat FeatureDescriptor;
typedef std::vector<Feature> FeatureList;
//typedef std::vector<FeatureDescriptor> DescriptorList;
typedef cv::Mat DescriptorList;
typedef std::vector<cv::DMatch> MatchList;

#endif /* OPENCV_INCLUDES_H_ */









// AUFRUFE !!! ////////////

//		switch(method)
//		{
//		case 0:
//		{
//			tvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			rvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			//cv::solvePnP(visible_controlpoints, controlpoints_images, cameraMatrix, distCoeffs, rvecs, tvecs, false, CV_ITERATIVE);
//			solvePnP_reimp(visible_controlpoints, controlpoints_images, cameraMatrix, cv::noArray(), rvecs, tvecs, false, CV_ITERATIVE);
//			break;
//		}
//		case 1:
//		{
//			tvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			rvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			cv::solvePnP(visible_controlpoints, controlpoints_images, cameraMatrix, cv::noArray(), rvecs, tvecs, false, CV_EPNP);
//#ifdef DEBUG
//			std::cout << "After EPnP: " << std::endl;
//			std::cout << rvecs.at<double>(0,0) << " " << tvecs.at<double>(0,0) << std::endl;
//			std::cout << rvecs.at<double>(1,0) << " " << tvecs.at<double>(1,0) << std::endl;
//			std::cout << rvecs.at<double>(2,0) << " " << tvecs.at<double>(2,0) << std::endl;
//#endif
//			solvePnP_reimp(visible_controlpoints, controlpoints_images, cameraMatrix, cv::noArray(), rvecs, tvecs, true, CV_ITERATIVE);
//			break;
//		}
//		case 2:
//		{
//			tvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			rvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			cv::solvePnP(visible_controlpoints, controlpoints_images, cameraMatrix, cv::noArray(), rvecs, tvecs, false, CV_EPNP);
//			break;
//		}
//		case 3:
//		{
//			if(!use_matrix)
//			{
//				tvecs = cv::Mat::zeros(3,1,CV_64FC1);
//				rvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			}
//			//cv::solvePnPRansac(cv::Mat(visible_controlpoints), cv::Mat(controlpoints_images), cameraMatrix, cv::noArray(), rvecs, tvecs, false, 500, reprojection_error, int(controlpoints_images.size()*ratio_inliers), cv::noArray(), CV_ITERATIVE);
//			cv::solvePnPRansac(cv::Mat(visible_controlpoints), cv::Mat(controlpoints_images), cameraMatrix, distCoeffs, rvecs, tvecs, use_matrix, 500, reprojection_error, int(controlpoints_images.size()*ratio_inliers), inliers, CV_EPNP);
//			break;
//		}
//		case 4:
//		{
//			if(!use_matrix)
//			{
//				tvecs = cv::Mat::zeros(3,1,CV_64FC1);
//				rvecs = cv::Mat::zeros(3,1,CV_64FC1);
//			}
//			//cv::solvePnPRansac(cv::Mat(visible_controlpoints), cv::Mat(controlpoints_images), cameraMatrix, cv::noArray(), rvecs, tvecs, false, 500, reprojection_error, int(controlpoints_images.size()*ratio_inliers), cv::noArray(), CV_ITERATIVE);
//			cv::solvePnPRansac(cv::Mat(visible_controlpoints), cv::Mat(controlpoints_images), cameraMatrix, distCoeffs, rvecs, tvecs, use_matrix, 500, reprojection_error, int(controlpoints_images.size()*ratio_inliers), inliers, CV_ITERATIVE);
//			break;
//		}
//		default:
//			cv::solvePnP(visible_controlpoints, controlpoints_images, cameraMatrix, distCoeffs, rvecs, tvecs, false, CV_ITERATIVE);
//			break;
//		}



//if(_wallis) {
//	cv::Mat syn_img;

//	WallisGrayscaleChannelOnly(synthetic_image, syn_img, _k);
//	//Wallis(synthetic_image, syn_img, _k);
//	WallisGrayscaleChannelOnly(reference_image, ref_img, _k);
//	//Wallis(reference_image, ref_img, _k);
//	syn_img.copyTo(synthetic_image);
//	ref_img.copyTo(reference_image);
//	syn_img.release();
//	ref_img.release();
//}