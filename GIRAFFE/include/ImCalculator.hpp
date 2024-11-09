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

	// Dtor
	~ImCalculator();

	/**
	 * @brief Initializes the ImCalculator with necessary parameters, including camera settings and directory setup.
	 *
	 * This function sets up the ImCalculator by initializing the log file, setting up distance limits,
	 * reserving space for pyramid masks, and configuring camera settings based on data from the
	 * provided DataManager. It also ensures that the output directory exists.
	 *
	 * @param _dataManager Pointer to the DataManager providing required parameters and file paths.
	 */
	void init(DataManager* _dataManager);
	
	
	/**
	 * @brief Projects a 3D point onto the 2D image plane and updates mask and image data accordingly.
	 *
	 * This function transforms a given 3D point from world coordinates into camera space,
	 * projects it onto the 2D image plane, and updates the pixel values and depth mask if
	 * the point is closer than any existing value at that pixel. Pyramid masks at lower
	 * resolutions are also updated to enable multi-scale processing.
	 *
	 * @param lp Pointer to the LaserPoint object containing the 3D coordinates and color information.
	 */
	void projectPoint(LaserPoint* lp);

	/**
	 * @brief Initializes the image with the given bounding box.
	 *
	 * This function calculates the image plane based on the bounding box and pixel size,
	 * then initializes the images (e.g., `_image`, `_mask`, `_distImage`, pyramids) with the
	 * calculated columns and rows.
	 *
	 * @param b The bounding box containing the minimum and maximum values for x, y, and z.
	 */
	void init_image(BoundingBox* b);

	/**
	 * @brief Generates output images based on distance and filtering operations.
	 *
	 * This function calculates and saves the original distance image, applies filtering
	 * to remove overlapping background pixels, and recalculates both the distance image
	 * and distance pyramids.
	 *
	 * @warning Call this function only after all points have been projected.
	 */
	void write_images();

	/**
	 * @brief Saves all generated synthetic images, including distance maps and pyramids, in the ImCalculator directory.
	 *
	 * This function handles the saving of synthetic images created during processing. It first releases
	 * and reinitializes the mask, writes the distance images and their pyramid levels to files,
	 * and saves the synthetic color image if available. If the image is in grayscale, it converts
	 * intensity values to an 8-bit format for storage.
	 *
	 * @details
	 * - Saves the distance image (`dist_image.png`) and distance pyramids (`dist_image_pX.png` for each level).
	 * - Saves the color or grayscale synthetic image in the directory as specified by `_working_dir_imcalculator`.
	 * - Logs each save operation and includes a check for whether the image is color or grayscale.
	 */
	void save_images();
	
	/**
	 * @brief Fills 2D, 3D, and color vectors with data from the coordinate image.
	 *
	 * This function retrieves synthetic 2D coordinates, 3D coordinates, and RGB colors from a
	 * coordinate image managed by the data manager. It checks that each vector has been initialized
	 * before use, clears any existing data, and then populates the vectors with new data from the
	 * coordinate image.
	 */
	void fill_vectors();

	/**
	 * @brief Fills masked regions in the input image with colors from provided synthetic points.
	 *
	 * This function generates a mask based on non-white pixels in the image, fills specified points
	 * with corresponding colors, and interpolates colors to fill in remaining masked regions.
	 *
	 * @param radius_mask_fill The radius used for filling the mask around non-white areas.
	 */
	void fill_image(int radius_mask_fill = 10);

	cv::Mat* get_image(){ return _image; }

	

private:

	const std::string TAG = "ImCalculator:\t";
	LogFile* logfile;

	/**
	 * @brief Calculates the image plane projection from the bounding box.
	 *
	 * This function computes the image plane by projecting the bounding box's coordinates in camera space
	 * and logs various details like camera position and rotation matrix.
	 *
	 * @param plane The array where the image plane corners (u0, v0, u1, v1) will be stored.
	 */
	void calc_image_plane(float* plane );

	/**
	 * @brief Initializes all necessary images with the given columns and rows.
	 *
	 * This function sets up the following images:
	 * - _image: The main image (initialized to white).
	 * - _mask: The mask image (initialized to -1.0f).
	 * - _distImage: The distance image (initialized to black).
	 * - next_dists: Distance pyramid images.
	 * - next_masks: Mask pyramid images.
	 *
	 * The pyramids are generated based on the input dimensions.
	 *
	 * @param column Number of columns for the image size.
	 * @param row Number of rows for the image size.
	 */
	void init_images(int column, int row);


	/**
	 * @brief Calculates the colors for `_distImage` based on distances in `_mask`.
	 *
	 * This function applies a color gradient to `_distImage` based on distance values in `_mask`,
	 * where distances are normalized to the range [0,1] using `d_min` and `d_diff`.
	 * Each distance is mapped to RGB values, with optional resetting of pixel values.
	 *
	 * @param d_min Minimum distance value used for normalization.
	 * @param d_diff Difference between the maximum and minimum distances for normalization.
	 * @param deleteCurrentData If true, resets pixels where no valid distance is found.
	 */
	void calc_dist_image(float d_min, float d_diff, bool deleteCurrentData = true);


	 /**
	  * @brief Filters out overlapping background pixels from the foreground using pyramid images.
	  *
	  * This function iterates through pyramid images and progressively removes background pixels
	  * that are further than a given threshold distance `db` from corresponding foreground pixels.
	  * Edge pixels are treated specially to avoid unintended deletion, with checks for neighboring
	  * pixels in the current pyramid level to ensure consistency.
	  *
	  * @param db Maximum allowed distance between a foreground and background pixel.
	  */
	void filter_image(float db);


	/**
	 * @brief Calculates and sets colors for each distance pyramid level based on distance values.
	 *
	 * This function applies a color gradient to each pixel in the `next_dists` images
	 * based on distance values stored in `next_masks`. Distances are normalized and mapped to a
	 * color blend: red, green, and blue intensities vary according to the distance value.
	 *
	 * @param d_min Minimum distance value for normalization.
	 * @param d_diff Difference between minimum and maximum distance values for normalization.
	 */
	void calc_dist_pyramids(float d_min, float d_diff);

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
	int _columns, _rows;

	// the minima and maxima distance in current Images
	float _dist_min, _dist_max;

	double* _rotM;
	float* _image_plane;
	BoundingBox* _frustum;
	DataManager* _data_manager;

	std::vector<cv::Point2d> _real_matched_pts;
	std::vector<cv::Point2d> _synth_matched_pts;
	std::vector<cv::Point2d> _image_points_real;
	std::vector<cv::Point2d> _image_points_synth;
	std::vector<cv::Point3d> _object_points;

	// fillImages
	// init 
	cv::Mat _maske_8UC1; // Maske für ungefüllte Bereiche --> vermeide hier SiftSuche nach Punkten
	cv::Mat _realImage;// echtes Bild als input
	cv::Mat _realImage_copy_orig;

	fs::path _working_dir_imcalculator;
	
};
