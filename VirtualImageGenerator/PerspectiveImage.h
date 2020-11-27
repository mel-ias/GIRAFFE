#pragma once

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include"opencv2\opencv.hpp"
#include <stdio.h>
#include <ctype.h>
#endif
#include <string>
#include <iostream>
#include <fstream>
#include <limits>

#include"BoundingBox.hpp"
#include"PointLoader.h"
#include"ImCalculator.hpp"
#include"DataManager.h"
#include"LogfilePrinter.h"

// class to generate a perspective mapping of the pointcloud
class PerspectiveImage{

public:

    // C'tor
    PerspectiveImage(DataManager* dataManager);

    // D'tor
    virtual ~PerspectiveImage();

    // generates the image from the point cloud
    void generateImage();

	// public Funktion zum Filtern von Bilddaten wenn gew�nscht!
	cv::Mat PerspectiveImage::applyFilterAlgorithms(
		const cv::Mat& imageToBeFiltered,
		DataManager::FILTER_APPS filterToBeApplied);



private:

	std::string const TAG = "PerspImage:\t";

    // checks the variables
    void checkVariables();
	
	cv::Mat _realImage;
	cv::Mat _synthImage;
	std::vector<cv::Point> capturePoint;// point coordinates, global variable;s
	std::vector<cv::Point2d> _waterlinePoint2d;
	int n = 0;
	
	DataManager* dataManager;

	// instance of pointloader
	PointLoader* _pointloader;

	// an Calculator that generates our images
	ImCalculator calculator;

	// if true then intensity value is used as gray value otherwise range value
	bool _writeIntensity;

	// log part
	LogFile* logfile;
	
};
