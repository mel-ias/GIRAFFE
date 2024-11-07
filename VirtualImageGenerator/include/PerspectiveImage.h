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


class PerspectiveImage{

public:

	/**
	 * @brief Constructor for the PerspectiveImage class.
	 *
	 * Initializes the PerspectiveImage with a DataManager instance, setting up logging, and initializing
	 * the point loader and other necessary components.
	 *
	 * @param _dataManager Pointer to a DataManager instance used for retrieving paths and settings.
	 */
    PerspectiveImage(DataManager* dataManager);

	/**
	 * @brief Destructor for the PerspectiveImage class.
	 *
	 * Cleans up dynamically allocated resources, specifically the point loader instance.
	 */
    virtual ~PerspectiveImage();

	/**
	 * @brief Generates a synthetic image based on the point cloud and bounding box data.
	 *
	 * Sets up the calculator with the point loader, initializes the bounding box, reads binary point
	 * cloud data, and then processes and saves the generated image.
	 */
    void generateImage();



private:

	std::string const TAG = "PerspImage:\t";  // Tag used for logging messages specific to PerspectiveImage
	
	cv::Mat _real_image; // Matrix to hold the real image captured by the system
	cv::Mat _synth_image; // Matrix to store the generated synthetic image
	std::vector<cv::Point> _captured_point; // Stores coordinates of captured points, used globally
	std::vector<cv::Point2d> _image_point;  // Holds 2D coordinates of image points to be referenced

	DataManager* _data_manager; // Pointer to DataManager for accessing application-wide data and configurations
	PointLoader* _point_loader; // Pointer to the PointLoader instance used for loading point cloud data
	ImCalculator _calculator; // Instance of ImCalculator, responsible for generating synthetic images

	LogFile* logfile; // Pointer to LogFile for logging messages during processing
	
};
