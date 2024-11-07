#pragma once

#include <iostream>
#include <fstream>  // for std::ifstream
#include <vector>
#include <thread>
#include <future>
#include <mutex>
#include <cfloat>
#include <string>

#include "LaserPoint.h"
#include "ImCalculator.hpp"
#include "BoundingBox.hpp"
#include "DataManager.h"
#include "LogfilePrinter.h"

#define _USE_MATH_DEFINES
#include <math.h>



// class to load a pointcloud
class PointLoader {
public:    

	/**
	 * @brief Constructor for the PointLoader class.
	 * Initializes the PointLoader with a path to the point cloud and sets up logging and shift values.
	 *
	 * @param path_pcl Path to the point cloud file.
	 * @param dataManager Pointer to the DataManager object for logging and accessing shift values.
	 */
    PointLoader(std::string path_pcl, DataManager* data_manager);

	/**
	 * @brief Destructor for the PointLoader class.
	 * Cleans up resources by deallocating the input stream pointer.
	 */
    virtual ~PointLoader();

	/**
	 * @brief Checks if the point cloud file has a ".pw" extension.
	 *
	 * @return True if the file extension is ".pw", false otherwise.
	 */
	bool check_filetype_pw();

	/**
	 * @brief Displays a progress bar in the console for a given percentage.
	 *
	 * @param percent The current percentage (0-100) to display in the progress bar.
	 */
	void display_progress_bar(int percent);

	/**
	 * @brief Reads a binary file containing point cloud data, applies filtering and transformations,
	 *        and projects the points in a multi-threaded manner.
	 *
	 * @return int Returns 0 if successful, or -1 if an error occurs.
	 * @throws std::logic_error If no perspective image or bounding box is found.
	 */
	int read_binary_file();

	// SETTER
	void set_imc(ImCalculator* imc) { _imc = imc; }
	void set_bb(BoundingBox* bb) { _bb = bb; }

private:
   
	LogFile* logfile; // Pointer to a LogFile object for logging messages and errors.
	const std::string TAG = "PointLoader:\t";
	std::string _path_point_cloud = ""; // File path to the point cloud data to be loaded.
    std::ifstream* _input_stream; // Pointer to an input file stream used to read binary data from the point cloud file.
	ImCalculator *_imc = nullptr; // Pointer to the perspective image calculator (ImCalculator) used to project points. Note: PointLoader does not own this pointer, so it should not delete it. Ensure that the object referenced by this pointer is managed elsewhere in the code.
	BoundingBox *_bb = nullptr; // Pointer to a BoundingBox object that defines the spatial limits for filtering points. Note: PointLoader does not own this pointer, so it should not delete it. The BoundingBox is used to determine which points are projected.
	double _shift_x, _shift_y, _shift_z; //Shift values for the x, y, and z coordinates, obtained from the DataManager.
};