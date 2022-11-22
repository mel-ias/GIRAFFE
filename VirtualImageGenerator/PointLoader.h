#pragma once

#include <iostream>
#include <vector>
#include "LaserPoint.h"
#include"ImCalculator.hpp"
#include"BoundingBox.hpp"
#include"DataManager.h"
#include"LogfilePrinter.h"

#define _USE_MATH_DEFINES
#include<math.h>


// class to load a pointcloud
class PointLoader {
public:
    // C'tor
    PointLoader(std::string filename, DataManager* dataManager);
    // D'tor
    virtual ~PointLoader();

	/*
	this reads a binary file (stored informations: id x y z intensity)
	In this method are only read the points to the pointcloud, that are in the visible space.
	*/
	int read_binary_file();

	//calculate the file size in bytes
	//void getFileSize(long long &size);
	long calculate_file_size(std::string path);

	/*checks, if the input file is a .pw file or not.*/
	bool check_file();

	bool have_read_color_points()const{ return color_points; }

	void set_imc(ImCalculator *imc){ my_imc=imc; }

	// reads the first byte and check if we have color points
	void check_color();

	void set_bb(BoundingBox *bb){ my_bb = bb; }


private:
   
	LogFile* logfile;
	const std::string TAG = "PointLoader:\t";
	
	std::string path_point_cloud = "";

    // input file stream
    std::ifstream* input_stream_ptr;

	// say, if the pointloader have read color points. else it will read intesity points
	bool color_points;

	/*a pointer to the perspective Image, that would be genrate on the base of this point Loader
	CAUTION PointLoader don't own this, so don't delete!*/
	ImCalculator *my_imc = nullptr;
	
	// the boundingBox needed to identify which point should be projected ( same as above don't delete !!)
	BoundingBox *my_bb = nullptr;

	// need DataManager for Shift x,y
	DataManager *dataManager = nullptr;

	double shift_x, shift_y;

	

};
