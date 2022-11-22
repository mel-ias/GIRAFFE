#pragma once

#ifndef IMCALCULATOR_H
#define IMCALCULATOR_H

#include"opencv2\opencv.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>
//#include"opencv2\nonfree\features2d.hpp"
#include"opencv2/flann/defines.h"
#include"LaserPoint.h"
#include<vector>
#include"CoordinateImage.hpp"
#include"BoundingBox.hpp"
#include"DLT.hpp"
#include"opencv_includes.h"
#include"fillImages.hpp"
#include"AccurateMatcher.h"
#include"Modell.h"
#include"plyloader.h"
#include"LogfilePrinter.h"
#include "Mathematics.h"


#include <iostream>
#include <string>
#include <algorithm>
#include "kdtree.h"
#include <omp.h>

#include "DataManager.h"

#include <Windows.h>

#endif


class ImCalculator
{

	


public:

	// Ctor
	ImCalculator() {

	};

	void init(DataManager* _dataManager);
	
	
	~ImCalculator();
	
	/*Projekt this point to the current images.
	CAUTION you should set all neccessary members befor (see setter of this class)*/
	void projectPoint(LaserPoint* lp);

	/*Init Images with the given bb*/
	void init_Image(BoundingBox* b);


	/*This do all the stuff, needed to generate the output images.
	CAUTION call this after all points are projected!!*/ 
	void writeImages();

	/*saves all generated Images*/
	void saveImages();

	
	bool haveProject(){ return haveProjectSomething; }

	void WriteColorPoints(bool color){ colorPoints = color; }

	void fillImage(size_t k_for_knn = 5); //Franks Algorithmus! wenn nichts eingegeben wurde nutze k_for_knn mit 5 als startwert


	cv::Mat get_FilledImage_Knn() { return _imageForImFill_inpaint_knn; } // _imageForImFill_knn
	cv::Mat get_FilledImage_Radius() { return _imageForImFill_inpaint_radius; }

	cv::Mat getMaske8UC1() { return _maske_8UC1; }

	void fillVektorsForImFill();


	cv::Mat* getMask(){ return _mask; }

	cv::Mat* getImage(){ return _image; }

	cv::Mat* getDistImage(){ return _distImage; }
	

private:

	const std::string TAG = "ImCalculator:\t";
	LogFile* logfile;

	/*This calculate the image Plane as an projektion from the bounding box far area.
	It also sets ie,je and ke*/
	void calc_image_Plane(float* plane );

	/*Initialize all nessecary images with the given coloumns and rows.
	The size of the pyramids will also be calculate.*/
	void init_images(int column, int row);

	/*This calcs the colors of the distImage with the current data in _mask.
	deleteCurrentData say if information in the distImage, which is deleted in_mask will be reset to 0 or not.*/
	void calc_distImage(float d_min, float d_diff, bool deleteCurrentData = true);

	/*Filter Methode, um überlagerte Hintergrund Pixel aus dem Vordergrund zu eliminieren.
	Hierfür werden die Pyramiden Bilder genutzt und neu die Hintergrund pixel schrittwiese gelöscht.
	db gibt den maximalen Abstand zwischen einem Vordergrund und einem Hintergrund Pixel an.
	Wenn dieser Abstand überschritten wird, wird das Hintergrund pixel gelöscht.*/
	void filter_image(float db);


	// comparator für Sortierung von Matches!
	bool response_comparator(const cv::DMatch& p1, const cv::DMatch& p2) {
		return p1.distance > p2.distance;
	}

	void colormappingIntensity(const cv::Mat& mat, const cv::Mat& maske, const cv::Rect& rectangleToCrop, const std::string& fileName, cv::Mat& adjMap, cv::Mat &falseColorsMap);


	/*Calc the color for all distance pyramids*/
	void calc_distPyramids(float d_min, float d_diff);

	// calculates the cross product between a and b, returns the solution in c
	void cross(float* a, float* b, float* c);

	// image of the projected intensity values
	cv::Mat* _image;

	// image mask indicating where points have been projected
	cv::Mat* _mask;

	// for the distance image
	cv::Mat* _distImage;

	// pyramid masks
	std::vector<cv::Mat*> next_masks;
	// pyramid dist images
	std::vector<cv::Mat*> next_dists;

	// pixel size
	float _pixSize;
	// distance between origin and image plane
	float _ck;

	//Bildgröße von Smartphone Bild
	cv::Size _imageSize;

	// the columns and rows in the current Images
	int columns, rows;

	// the minima and maxima distance in current Images
	float distMin, distMax;

	float* rot_xyz;

	float image_plane[4];

	BoundingBox* bb;

	// say if ImCalculator have project one or more points
	bool haveProjectSomething;

	// say if color points will be projected
	bool colorPoints;

	// gefüllte und zugeschnittene synthetische Bilder --> Output von fillImage
	cv::Mat _imageForImFill_knn_Crop;
	cv::Mat _imageForImFill_inpaint_radius;
	cv::Mat _imageForImFill_inpaint_knn;


	//-- Localize the object, gute Matches zwischen den Bildern!
	std::vector<cv::Point2d> real_matched_pts;
	std::vector<cv::Point2d> synth_matched_pts;


	// für zuordnung von keypoint, gefunden in synthetischen und echten bild und für guten match befunden und
	// projizierten punkt in punktwolke --> liegen schließlich nicht zwangsläufig aufeinander, wenn interpolierter
	// punkt von filling getroffen wurde. Mache distanzkritierum! --> nur wenn dist < halbes pixel, nehme diesen punkt als 
	// entsprechenden keypoint an und hole den passenden 3D Wert dazu.

	std::vector<cv::Point2d> imagePoints_real;
	std::vector<cv::Point2d> imagePoints_synth;
	std::vector<cv::Point3d> objectPoints;

	//fillImages
	// init 
	cv::Mat adjMap; // Graustufenbild
	cv::Mat falseColorsMap; // falschfarbenBild

	cv::Mat _maske_8UC1; // Maske für ungefüllte Bereiche --> vermeide hier SiftSuche nach Punkten
	
	// echtes Bild als input
	cv::Mat _realImage;
	cv::Mat _realImage_copy_orig;

	// Camera internals
	double focal_length = 0; // real.cols; // Approximate focal length.
	cv::Point2d center = cv::Point2d(0, 0);
	cv::Mat camera_matrix;//= (cv::Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	cv::Mat camera_matrix_ideal;
	cv::Mat dist_coeffs;// = cv::Mat::zeros(4, 1, CV_32FC1); // Assuming no lens distortion

	cv::Mat T; //transformationsmatrix

	DataManager* dataManager;

	std::string path_directory_ImCalculator;
	
};
