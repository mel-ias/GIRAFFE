#include "ImCalculator.hpp"
#include <limits>
#include "opencv2\photo.hpp"
#include "DataManager.h"


ImCalculator::ImCalculator() {
	_pixSize = 0.0f;
	_image = nullptr;
	_mask = nullptr;
	_distImage = nullptr;
	_pixSize = 0.0f;
	_ck = 0.0f;
	_frustum = nullptr;
	_columns = 0;
	_rows = 0;
	logfile = nullptr;
	_rotM = new double [9];
	_image_plane = new float [4];
	_dist_min = 0.0f;
	_dist_max = 0.0f;
	_data_manager = nullptr;
}


void ImCalculator::init(DataManager* _dataManager) {

	// Initialize logfile and log the start of initialization
	logfile = _dataManager->get_logfile();
	logfile->append("");
	logfile->append(TAG + "---- Initializing Image Calculator ----");

	// Assign data manager and initialize other members
	_data_manager = _dataManager;
	_mask = nullptr;
	_image = nullptr;

	_dist_max = 0.0f;
	_dist_min = std::numeric_limits<float>::max();
	_frustum = nullptr;

	// Reserve space for next masks and distance pyramids
	next_masks.reserve(3);
	next_dists.reserve(3);

	// Retrieve and log camera settings
	_ck = _data_manager->get_principal_distance();
	_imageSize = _data_manager->get_size_true_image();
	_pixSize = _data_manager->get_pixel_size();
	logfile->append(TAG + "Camera parameters received (ck, imageSize, pixSize).");

	// Reference and copy the real image
	_realImage = _data_manager->get_true_image();
	_realImage.copyTo(_realImage_copy_orig);
	logfile->append(TAG + "Real image received and original copy created.");

	// Set up the ImCalculator working directory
	_working_dir_imcalculator = _data_manager->get_path_working_directory() / "ImCalculator";
	if (!fs::exists(_working_dir_imcalculator)) {
		fs::create_directory(_working_dir_imcalculator);
	}
	logfile->append(TAG + "ImCalculator directory created or verified: " + _working_dir_imcalculator.string());
}



ImCalculator::~ImCalculator()
{
	if (_mask != nullptr) _mask->release();
	if (_image != nullptr) _image->release();

	// release the pyramids
	if (!next_masks.empty()) {
		for (unsigned int i = 0u; i < next_masks.size(); ++i)
			next_masks[i]->release();
	}

	if (!next_dists.empty()) {
		for (unsigned int i = 0u; i < next_dists.size(); ++i)
			next_dists[i]->release();
	}
}


// saves all generated synthetic images in ImCalculator directory
void ImCalculator::save_images() {

	logfile->append(TAG + "Saving synthetic images...");

	// Release and reinitialize the mask as an 8-bit single-channel image
	_mask->release();
	_mask = new cv::Mat(_image->rows, _image->cols, CV_8UC1);

	// Write the main distance image
	cv::imwrite((fs::path(_working_dir_imcalculator) / "dist_image.png").string(), *_distImage);

	// Save distance pyramids if they exist
	for (size_t i = 0; i < next_dists.size(); ++i) {
		std::string name = "dist_image_p" + std::to_string(i) + ".png";
		cv::imwrite((fs::path(_working_dir_imcalculator) / name).string(), *next_dists[i]);
	}

	// Check if _image is an 8-bit color image and save it directly
	if (_image->depth() == CV_8U && _image->channels() == 3) {
		cv::imwrite((fs::path(_working_dir_imcalculator) / (_data_manager->get_filename_true_image() + "_synth.png")).string(), *_image);
		logfile->append(TAG + "Saved synthetic RGB image.");
		return; // No further processing required for color images
	}

	// Clear mask for grayscale conversion
	_mask->setTo(cv::Scalar(0));

	// Convert intensity values to 8-bit grayscale and store in _mask
	for (int i = 0; i < _image->rows; ++i) {
		for (int j = 0; j < _image->cols; ++j) {
			// Scale float intensity value to 8-bit range
			float intensity = _image->ptr<float>(i)[j];
			_mask->at<uchar>(i, j) = static_cast<uchar>(std::clamp(intensity * 255.0f + 0.5f, 0.0f, 255.0f));
		}
	}

	// Save the grayscale intensity mask
	cv::imwrite((fs::path(_working_dir_imcalculator) / (_data_manager->get_filename_true_image() + ".png")).string(), *_mask);
	logfile->append(TAG + "Saved synthetic grayscale intensity image.");
}



// Project the given 3D point to the current image and update the mask and image data as necessary.
void ImCalculator::projectPoint(LaserPoint* lp) {
	// Compute the difference between the laser point's world coordinates and the camera's position in world space
	double dx = lp->_xyz[0] - _frustum->get_X0_Cam_World()[0];
	double dy = lp->_xyz[1] - _frustum->get_X0_Cam_World()[1];
	double dz = lp->_xyz[2] - _frustum->get_X0_Cam_World()[2];

	// Transform the coordinates to camera space using the rotation matrix
	double x = _rotM[0] * dx + _rotM[3] * dy + _rotM[6] * dz;
	double y = _rotM[1] * dx + _rotM[4] * dy + _rotM[7] * dz;
	double z = _rotM[2] * dx + _rotM[5] * dy + _rotM[8] * dz;

	// Check if the point is within the acceptable near/far bounds
	if (z < _data_manager->get_min_dist_to_X0() || z > _frustum->get_dist()) return;

	// Project 3D camera-space coordinates to the 2D image plane
	double u = _ck * x / z;
	double v = _ck * y / z;

	// Convert projected coordinates to image pixel coordinates
	double column_float = (u - _image_plane[0]) / _pixSize;
	double row_float = (v - _image_plane[2]) / _pixSize;

	// Calculate integer row and column indices
	int column = static_cast<int>(floor(column_float));
	int row = static_cast<int>(floor(row_float));

	// Ensure the pixel is within image boundaries
	if (row < 0 || row >= _rows || column < 0 || column >= _columns) return;

	// Compute the Euclidean distance from the camera to the point
	float dist = static_cast<float>(sqrt(dx * dx + dy * dy + dz * dz));

	// Update the min and max distance values
	if (dist < _dist_min) _dist_min = dist;
	if (dist > _dist_max) _dist_max = dist;

	// Retrieve the current mask value at this pixel position
	float& mask_value = _mask->ptr<float>(row)[column];
	cv::Point3_<uchar>* pixel = _image->ptr<cv::Point3_<uchar>>(row, column);

	// Update mask and image if the new point is closer, or if there's no existing mask value
	if (mask_value > 0.0f) {
		if (dist < mask_value) {
			mask_value = dist;
			pixel->x = lp->color[2];
			pixel->y = lp->color[1];
			pixel->z = lp->color[0];
			_data_manager->get_coordinate_image()->set_pixel(
				column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float)
			);
		}
	}
	else {
		// Assign new values if no previous mask data exists at this pixel
		mask_value = dist;
		pixel->x = lp->color[2];
		pixel->y = lp->color[1];
		pixel->z = lp->color[0];
		_data_manager->get_coordinate_image()->set_pixel(
			column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float)
		);
	}

	// Update pyramid masks for additional resolution levels
	for (unsigned int p = 0; p < next_masks.size(); ++p) {
		float scale = powf(2.0f, static_cast<float>(p + 1));

		// Calculate the scaled coordinates in the pyramid level
		int cPyr = static_cast<int>(floor((u - _image_plane[0]) / (scale * _pixSize)));
		int rPyr = static_cast<int>(floor((v - _image_plane[2]) / (scale * _pixSize)));

		// Ensure pyramid coordinates are within bounds before accessing the mask
		if (rPyr < 0 || rPyr >= next_masks[p]->rows || cPyr < 0 || cPyr >= next_masks[p]->cols) continue;

		float& mask_pix = next_masks[p]->ptr<float>(rPyr)[cPyr];

		// Update pyramid mask if it's empty or if the new distance is closer
		if (mask_pix > 0.0f) {
			if (dist < mask_pix) mask_pix = dist;
		}
		else {
			mask_pix = dist;
		}
	}
}


void ImCalculator::init_image(BoundingBox* b) {

	// Set the bounding box (bb) to the given bounding box
	_frustum = b;

	// Calculate the image plane based on the bounding box
	calc_image_plane(_image_plane);

	// Calculate the number of columns and rows based on the image plane and pixel size
	_columns = static_cast<int>(ceil((_image_plane[1] - _image_plane[0]) / _pixSize)) + 1; // X dimension
	_rows = static_cast<int>(ceil((_image_plane[3] - _image_plane[2]) / _pixSize)) + 1; // Y dimension

	// Initialize the images with the calculated dimensions
	init_images(_columns, _rows);

	// Log the initialization of the image plane with the pixel dimensions
	logfile->append(TAG + "initialize image_plane (px): " + std::to_string(_columns) + "x" + std::to_string(_rows));
}


void ImCalculator::calc_image_plane(float* plane) {

	// Coordinates in camera system (bb represents bounding box in camera space)
	float xk, yk, zk;

	// Upper left corner (u0, v0)
	xk = _frustum->get_xMin();
	yk = -_frustum->get_zMax();  // Inverting Z for the camera space convention
	zk = _frustum->get_yMax();   // Inverted Y for the camera system

	plane[0] = xk / zk * _ck;  // u0
	plane[2] = yk / zk * _ck;  // v0

	// Lower right corner (u1, v1)
	xk = _frustum->get_xMax();
	yk = -_frustum->get_zMin();  // Inverting Z again
	plane[1] = xk / zk * _ck;  // u1
	plane[3] = yk / zk * _ck;  // v1

	// Get the rotation matrix from dataManager
	_rotM = _data_manager->get_rotM();

	// Logging information
	logfile->append(TAG + "TVec:");
	logfile->append("\t\t" + std::to_string(_frustum->get_X0_Cam_World()[0]) + " " +
		std::to_string(_frustum->get_X0_Cam_World()[1]) + " " +
		std::to_string(_frustum->get_X0_Cam_World()[2]));

	logfile->append(TAG + "RotM:");
	logfile->append("\t\t" + std::to_string(_rotM[0]) + " " + std::to_string(_rotM[3]) + " " +
		std::to_string(_rotM[6]), 4);
	logfile->append("\t\t" + std::to_string(_rotM[1]) + " " + std::to_string(_rotM[4]) + " " +
		std::to_string(_rotM[7]), 4);
	logfile->append("\t\t" + std::to_string(_rotM[2]) + " " + std::to_string(_rotM[5]) + " " +
		std::to_string(_rotM[8]), 4);
	logfile->append("");
	logfile->append(TAG + "calculated plane: " + std::to_string(plane[0]) + "," + std::to_string(plane[1]) + "," +
		std::to_string(plane[2]) + "," + std::to_string(plane[3]), 4);
}



void ImCalculator::init_images(int column, int row) {

	// Initialize the main image (_image) with white color (255, 255, 255)
	_image = new cv::Mat(row, column, CV_8UC3);
	_image->setTo(cv::Scalar(255, 255, 255));

	// Initialize the mask (_mask) with default value -1.0f
	_mask = new cv::Mat(row, column, CV_32FC1);
	_mask->setTo(cv::Scalar(-1.0f));

	// Initialize the distance image (_distImage) with black color (0, 0, 0)
	_distImage = new cv::Mat(row, column, CV_8UC3);
	_distImage->setTo(cv::Scalar(0u, 0u, 0u));

	// Generate 3 pyramids for next_dists and next_masks
	for (int i = 1; i <= 3; ++i) {
		int scale = static_cast<int>(powf(2.0f, static_cast<float>(i)));  // Compute scaling factor for pyramid levels
		int pc = column % scale == 0 ? (column / scale) : (column / scale + 1);  // Calculate columns in pyramid
		int pr = row % scale == 0 ? (row / scale) : (row / scale + 1);  // Calculate rows in pyramid

		// Create and initialize distance pyramid image
		next_dists.push_back(new cv::Mat(pr, pc, CV_8UC3));
		next_dists[i - 1]->setTo(cv::Scalar(0, 0, 0));

		// Create and initialize mask pyramid image
		next_masks.push_back(new cv::Mat(pr, pc, CV_32FC1));
		next_masks[i - 1]->setTo(cv::Scalar(-1.0f));
	}

	// Set the coordinate image in the data manager
	_data_manager->set_coordinate_image(column, row);
}



void ImCalculator::write_images() {

	// Calculate the effective distance range for normalization
	_dist_max -= _dist_min;

	// Calculate the distance image without resetting existing data
	calc_dist_image(_dist_min, _dist_max, false);

	// Save the initial distance image
	cv::imwrite(fs::path(_working_dir_imcalculator / "dist_image_orig.png").string(), *_distImage);
	logfile->append(TAG + "calc new masks");

	// Apply filtering on the image to remove overlapping background pixels
	filter_image(1.0f);

	// Recalculate the distance image with resetting enabled (default `deleteCurrentData = true`)
	calc_dist_image(_dist_min, _dist_max);

	// Calculate the distance pyramids to generate layered color representations
	calc_dist_pyramids(_dist_min, _dist_max);
}



void ImCalculator::calc_dist_image(float d_min, float d_diff, bool deleteCurrentData) {

	logfile->append(TAG + "save distances.. ");

	// Iterate through each pixel in `_distImage`
	for (int r = 0; r < _distImage->rows; ++r) {
		for (int c = 0; c < _distImage->cols; ++c) {

			// Access the pixel at position (r, c) in `_distImage`
			cv::Point3_<uchar>* pixel = _distImage->ptr<cv::Point3_<uchar>>(r, c);

			// Retrieve the corresponding distance value from `_mask`
			float d = _mask->ptr<float>(r)[c];

			// Calculate normalized distance and set color if valid
			if (d > 0.0f) {
				d = (d - d_min) / d_diff; // Normalize `d` to [0, 1]

				// Calculate RGB values based on normalized `d`
				float red = (d <= 0.5f) ? (1.0f - 2.0f * d) : 0.0f;
				float green = (d <= 0.5f) ? (2.0f * d) : (2.0f - 2.0f * d);
				float blue = (d >= 0.5f) ? (2.0f * d - 1.0f) : 0.0f;

				// Set RGB values in `_distImage`, scaling to 0–255
				pixel->x = static_cast<unsigned char>(blue * 255u);
				pixel->y = static_cast<unsigned char>(green * 255u);
				pixel->z = static_cast<unsigned char>(red * 255u);

			}
			else if (deleteCurrentData) {
				// Reset pixel color to black (0,0,0) if `deleteCurrentData` is true and `d` is invalid
				pixel->x = 0u;
				pixel->y = 0u;
				pixel->z = 0u;
			}
		}
	}

	logfile->append(TAG + "done.");
}



void ImCalculator::filter_image(float db) {

	// Iterate through the pyramid levels from top to bottom (starting from highest resolution)
	for (int pyr_index = next_masks.size() - 1; pyr_index >= 0; --pyr_index) {

		// Current and next level masks in the pyramid
		cv::Mat* current_pyr = next_masks[static_cast<size_t>(pyr_index)];
		cv::Mat* nextPyr = (pyr_index != 0) ? next_masks[static_cast<size_t>(pyr_index - 1)] : _mask;

		// Traverse each pixel in the current pyramid level
		for (int r = 0; r < nextPyr->rows; ++r) {
			for (int c = 0; c < nextPyr->cols; ++c) {

				float d = nextPyr->ptr<float>(r)[c];
				if (d == -1.0f) continue;  // Skip invalid pixels

				// Coordinates in the current pyramid for half-resolution (downsampled) check
				int cPyr = c / 2;
				int rPyr = r / 2;

				// Distance at the downsampled position in the current pyramid
				float dPyr = current_pyr->ptr<float>(rPyr)[cPyr];

				// Check if the distance exceeds the threshold; if so, evaluate deletion criteria
				if (abs(d - dPyr) > db) {

					bool checkR = true;
					bool checkC = true;
					float dNew = 0.0f;

					// Check the row edge cases for even/odd rows
					if (r > 0 && r < nextPyr->rows - 1) {
						if (r % 2 == 0) {
							dNew = current_pyr->ptr<float>(rPyr - 1)[cPyr - 1];
						}
						else {
							dNew = current_pyr->ptr<float>(rPyr + 1)[cPyr + 1];
						}
						if (dNew > 0.0f) {
							checkR = abs(d - dNew) > db;
						}
					}

					// Check the column edge cases for even/odd columns
					if (c > 0 && c < nextPyr->cols - 1) {
						if (c % 2 == 0) {
							dNew = current_pyr->ptr<float>(rPyr)[cPyr - 1];
						}
						else {
							dNew = current_pyr->ptr<float>(rPyr)[cPyr + 1];
						}
						if (dNew > 0.0f) {
							checkC = abs(d - dNew) > db;
						}
					}

					// If not an edge pixel, delete it from the next pyramid mask
					if (checkR && checkC) {
						nextPyr->ptr<float>(r)[c] = -1.0f;

						// For the base pyramid level, update the color to indicate deletion
						if (pyr_index == 0 && r > 0 && r < _image->rows && c > 0 && c < _image->cols) {
							cv::Point3_<uchar>* pixel = _image->ptr<cv::Point3_<uchar>>(r, c);
							pixel->x = 255u; pixel->y = 255u; pixel->z = 255u;

							// Remove the corresponding pixel from the coordinate image in dataManager
							_data_manager->get_coordinate_image()->delete_pixel(c, r);
						}
					}
				}
			}
		}
	}
}



void ImCalculator::calc_dist_pyramids(float d_min, float d_diff) {

	// Iterate through each level of the distance pyramids
	for (unsigned int p = 1u; p <= next_dists.size(); ++p) {
		for (int r = 0; r < next_dists[static_cast<size_t>(p - 1u)]->rows; ++r) {
			for (int c = 0; c < next_dists[static_cast<size_t>(p - 1u)]->cols; ++c) {

				cv::Point3_ <uchar>* pixel = next_dists[static_cast<size_t>(p - 1u)]->ptr<cv::Point3_<uchar>>(r, c);

				// Retrieve the distance at the current pixel
				float d = next_masks[static_cast<size_t>(p - 1u)]->ptr<float>(r)[c];

				// Normalize distance to [0, 1] range, if greater than 0 (ignoring default/invalid values)
				if (d > 0) {
					d = (d - d_min) / d_diff;

					// Calculate color channels based on distance
					// Red decreases from 1 to 0 as d goes from 0 to 0.5, otherwise remains 0
					float red = d <= 0.5f ? (1.0f - 2.0f * d) : 0.0f; 

					// Green increases from 0 to 1 as d goes from 0 to 0.5, then decreases to 0 as d goes from 0.5 to 1
					float green = d <= 0.5f ? 2.0f*d : (2.0f - 2.0f*d); 

					// Blue remains 0 until d reaches 0.5, then increases to 1 as d approaches 1
					float blue = d >= 0.5f ? (2.0f*d - 1.0f) : 0.0f;

					// Assign calculated color values to the pixel, scaling to [0, 255] for uchar format
					pixel->x = static_cast<unsigned char>(blue * 255u);
					pixel->y = static_cast<unsigned char>(green * 255u);
					pixel->z = static_cast<unsigned char>(red * 255u);
				}
			}
		}
	}
}



void ImCalculator::fill_vectors() {
	// Retrieve vectors from the data manager
	std::vector<cv::Point2d>* synth2DCoordinates = _data_manager->get_pts_synth_2D_double();
	std::vector<cv::Scalar>* synth3DColors = _data_manager->get_pts_color_RGB_int();
	std::vector<cv::Point3d>* synth3DCoordinates = _data_manager->get_pts_synth_3D_double();

	// Check if vectors have been initialized; if not, exit the function
	if (synth2DCoordinates == nullptr || synth3DColors == nullptr || synth3DCoordinates == nullptr) {
		logfile->append(TAG + "Vectors for 2D points, 3D points, and color saving not initialized! Returning.");
		return;
	}

	// Clear vectors if they contain any existing points
	if (!synth2DCoordinates->empty()) {
		synth2DCoordinates->clear();
		synth3DColors->clear();
		synth3DCoordinates->clear();
	}

	// Retrieve pixel coordinates from the coordinate image
	std::vector<std::shared_ptr<CoordinateImage::Coordinate>> pixelCoordinates = _data_manager->get_coordinate_image()->getPixels();
	auto endIt = pixelCoordinates.cend();


	// Iterate through each pixel coordinate
	for (auto it = pixelCoordinates.cbegin(); it != endIt; ++it) {
		// Check if the current pixel coordinate is not null
		if (*it) { // Using the shared_ptr's conversion to bool
			// Add 2D coordinates to the vector
			synth2DCoordinates->emplace_back(cv::Point2d((*it)->xi, (*it)->yi));
			// Add 3D coordinates to the vector
			synth3DCoordinates->emplace_back(cv::Point3d((*it)->x, (*it)->y, (*it)->z));
			// Add RGB color to the vector
			synth3DColors->emplace_back(
				cv::Scalar(
					static_cast<unsigned int>((*it)->color[0]),
					static_cast<unsigned int>((*it)->color[1]),
					static_cast<unsigned int>((*it)->color[2])
				)
			);
		}
	}
}



void ImCalculator::fill_image(int radius_mask_fill) {
	logfile->append(TAG + "Fill Images RGB ...");

	// Initialize the mask with black (all zeroes)
	_maske_8UC1 = cv::Mat::zeros(_image->size(), CV_8UC1);
	for (int r = 0; r < _image->rows; r++) {
		for (int c = 0; c < _image->cols; c++) {
			if (_image->at<cv::Vec3b>(r, c) != cv::Vec3b(255, 255, 255)) {
				cv::circle(_maske_8UC1, cv::Point(c, r), radius_mask_fill, cv::Scalar(255), -1);
			}
		}
	}

	// Save initial mask for debugging/verification
	cv::imwrite(fs::path(_working_dir_imcalculator / "maskefill.png").string(), _maske_8UC1);

	cv::Mat tempBild = _image->clone(); // Temporary image to work on filled colors
	cv::Mat fillMask = cv::Mat::zeros(_maske_8UC1.size(), CV_8U); // Tracking filled areas

	// Populate fillMask and tempBild with synthetic point colors
	for (int i = 0; i < _data_manager->get_pts_synth_2D_double()->size(); ++i) {
		int x = static_cast<int>(_data_manager->get_pts_synth_2D_double()->at(i).x);
		int y = static_cast<int>(_data_manager->get_pts_synth_2D_double()->at(i).y);

		// Ensure the points are within image bounds
		if (x >= 0 && x < tempBild.cols && y >= 0 && y < tempBild.rows) {
			cv::Vec3b color{
				static_cast<uchar>(_data_manager->get_pts_color_RGB_int()->at(i)[0]),
				static_cast<uchar>(_data_manager->get_pts_color_RGB_int()->at(i)[1]),
				static_cast<uchar>(_data_manager->get_pts_color_RGB_int()->at(i)[2])
			};

			// Update pixel color and mark location in fillMask
			tempBild.at<cv::Vec3b>(y, x) = color;
			fillMask.at<uchar>(y, x) = 255;	
		}
	}

	// Interpolate colors for gaps in the masked area
	for (int row = 0; row < _image->rows; ++row) {
		for (int col = 0; col < _image->cols; ++col) {
			if (_maske_8UC1.at<uchar>(row, col) > 0 && fillMask.at<uchar>(row, col) == 0) {
				cv::Vec3f interpolierteFarbe = cv::Vec3f(0, 0, 0);
				int nCount = 0;

				// Average color from 8-neighbors to fill gaps
				for (int yOff = -1; yOff <= 1; ++yOff) {
					for (int xOff = -1; xOff <= 1; ++xOff) {
						int ny = row + yOff;
						int nx = col + xOff;

						if (ny >= 0 && ny < _image->rows && nx >= 0 && nx < _image->cols &&
							fillMask.at<uchar>(ny, nx) == 255) {
							cv::Vec3b neighborColor = tempBild.at<cv::Vec3b>(ny, nx);
							interpolierteFarbe += cv::Vec3f(neighborColor[0], neighborColor[1], neighborColor[2]);
							nCount++;
						}
					}
				}

				// Apply the averaged color if any neighbors were found
				if (nCount > 0) {
					interpolierteFarbe /= nCount;
					tempBild.at<cv::Vec3b>(row, col) = cv::Vec3b(
						cv::saturate_cast<uchar>(interpolierteFarbe[0]),
						cv::saturate_cast<uchar>(interpolierteFarbe[1]),
						cv::saturate_cast<uchar>(interpolierteFarbe[2])
					);
					fillMask.at<uchar>(row, col) = 255; // Mark as filled
				}
			}
		}
	}

	// Assign black to any remaining unfilled mask regions
	for (int row = 0; row < _image->rows; ++row) {
		for (int col = 0; col < _image->cols; ++col) {
			if (_maske_8UC1.at<uchar>(row, col) > 0 && fillMask.at<uchar>(row, col) == 0) {
				tempBild.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
			}
		}
	}

	// Apply morphological erosion to refine edges of the filled area
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius_mask_fill, radius_mask_fill));
	cv::erode(_maske_8UC1, _maske_8UC1, erodeElement);
	tempBild.setTo(cv::Scalar(0, 0, 0), _maske_8UC1 == 0); // Set areas outside mask to black

	// Update the original image with filled regions
	tempBild.copyTo(*_image);

	// Save the final filled image for verification
	cv::imwrite(fs::path(_working_dir_imcalculator / "filled_image.png").string(), *_image);
	logfile->append(TAG + "write filled_image.png");
}