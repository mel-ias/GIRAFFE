#pragma once
#include<vector>
#include<string>
#include<cstdint>
#include"opencv2\opencv.hpp"
#include"LaserPoint.h"

/**
* This is class, which is used to map the coordinates of each point from the point cloud to his projected pixel in the virtual image.
* In this project it only needs to read and write data (!)
**/
class CoordinateImage
{
public:

#pragma pack(push,1)
	struct CimHeader
	{
		uint32_t width;
		uint32_t height;
		double x0;
		double y0;
		double z0;
		float ps;
		float* r;
	};
#pragma pack(pop)


	/*Saving structure for the image pixel (x,y and z coordinates).*/
	struct Coordinate{

		double x, y, z;
		unsigned char color[3];
		float xi, yi;


		Coordinate(){ 
			x = y = z = 0.0; 
			color[0] = 0;
			color[1] = 0;
			color[2] = 0;
			xi = yi = 0;
		}

		Coordinate(double xc, double yc, double zc, cv::Point3_ <uchar>* colorc, float xic, float yic){
			x = xc; y = yc; z = zc; 
			color[0] = colorc->x; // [0];
			color[1] = colorc->y; // [1];
			color[2] = colorc->z; // [2];
			xi = xic;
			yi = yic;
		}

	};

	CoordinateImage() = delete;

	CoordinateImage(const int width, const int height);
	~CoordinateImage();

	CoordinateImage(CoordinateImage &src) = delete;
	CoordinateImage& operator=(CoordinateImage &rhs) = delete;


	/*This returns a constant pointer to the Coordinates of the given pixel.
	That mean you can change the Coordinates but not the Pointer. (The memory pointers are at the control of this class!).
	Use setPixel to ensure, that a needed pointer will be created.*/
	Coordinate * const get_pixel(const int column, const int row);

	Coordinate * const get_pixel(const int column, const int row)const;

	/*Change the data at the given pixel to the given coordinates.*/
	void set_pixel(const int column, const int row, const Coordinate &c);

	// delete the Data at the given Pixel
	void delete_pixel(const int column, const int row);

	int getWidth(){ return width; }

	int getHeight(){ return height; }

	std::vector<Coordinate*> getPixels() { return pixels; }

private:
	
	int width,height;
	std::vector<Coordinate*> pixels;

	std::stringstream logfile;

};

