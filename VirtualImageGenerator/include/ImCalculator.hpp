#pragma once

#ifndef IMCALCULATOR_H
#define IMCALCULATOR_H


#include"LaserPoint.h"
#include"CoordinateImage.hpp"
#include"BoundingBox.hpp"
#include"LogfilePrinter.h"
#include"DataManager.h"

#include <iostream>
#include <string>
#include <algorithm>
#include <omp.h>
#include <vector>
#include <Windows.h>

#include <opencv2/opencv.hpp>
#include <opencv2/flann.hpp>
#include <iostream>
#include <string>
#include <algorithm>
#include <omp.h>

#endif


#ifdef min
#undef min
#endif

#ifdef max
#undef max
#endif


class ImCalculator
{

	


public:

	// Ctor
	ImCalculator();

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
	
	void fill_vectors();
	void fill_image(int radius_mask_fill = 10);

	cv::Mat* getImage(){ return _image; }

	

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


	/*Calc the color for all distance pyramids*/
	void calc_distPyramids(float d_min, float d_diff);

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

	double* rot_xyz;
	float* image_plane;
	BoundingBox* bb;
	DataManager* dataManager;

	// für zuordnung von keypoint, gefunden in synthetischen und echten bild und für guten match befunden und
	// projizierten punkt in punktwolke --> liegen schließlich nicht zwangsläufig aufeinander, wenn interpolierter
	// punkt von filling getroffen wurde. Mache distanzkritierum! --> nur wenn dist < halbes pixel, nehme diesen punkt als 
	// entsprechenden keypoint an und hole den passenden 3D Wert dazu.
	std::vector<cv::Point2d> real_matched_pts;
	std::vector<cv::Point2d> synth_matched_pts;
	std::vector<cv::Point2d> imagePoints_real;
	std::vector<cv::Point2d> imagePoints_synth;
	std::vector<cv::Point3d> objectPoints;

	// fillImages
	// init 
	cv::Mat _maske_8UC1; // Maske für ungefüllte Bereiche --> vermeide hier SiftSuche nach Punkten
	cv::Mat _realImage;// echtes Bild als input
	cv::Mat _realImage_copy_orig;

	fs::path _working_dir_imcalculator;
	
};
