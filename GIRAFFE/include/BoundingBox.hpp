#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <sstream>
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <cfloat>
#include "CameraPose.hpp"
#include "CameraIntrinsics.hpp"

#include "LogFilePrinter.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif


class BoundingBox
{
public:

	/**
	 * @brief Constructor for the ViewFrustum class.
	 * Initializes the view frustum parameters and logs the initialization steps.
	 *
	 * @param logfile Pointer to a LogFile object for logging messages.
	*/
	BoundingBox(LogFile* logfile);

	/**
	 * @brief Destructor for the ViewFrustum class.
	 * Cleans up allocated memory for camera position and rotation matrix.
	 */
	~BoundingBox() = default;

	/**
	 * @brief Updates the view frustum based on the provided camera pose.
	 *
	 * This function calculates the half-width and half-height of the far plane in camera coordinates,
	 * including offsets for uncertainty (bounding box). It also computes the axis-aligned bounding box
	 * in world coordinates based on the camera pose and updates the internal state of the view frustum.
	 * This requires that the view angles, frustum depth, and offsets have already been set using the appropriate setter functions.
	 *
	 * @param pose The CameraPose object representing the camera's position and orientation in world coordinates.
	 */
	void update(const CameraPose& pose);

	/**
	 * @brief Set the depth of the view frustum.
	 *
	 * This function sets the depth of the view frustum to the given distance from the camera.
	 * All points within the specified distance from the camera will be inside the view frustum.
	 *
	 * @param distance The distance from the camera, defining the new depth of the view frustum.
	 */
	void set_frustum_depth(double distance) {
		_d = distance;
	}

	/**
	 * @brief Set the horizontal half-width of the bounding box.
	 *
	 * This function sets the horizontal half-width `half_width`, which represents half of the width
	 * of the bounding box close to the camera's projection center, corresponding to the horizontal
	 * extent of the bounding box or frustum in the horizontal plane.
	 *
	 * @param half width The horizontal half-width, typically derived from GPS accuracy or frustum width.
	 */
	void set_bb_offset_X0_xy(double half_width) {
		_bb_offset_X0_xy = half_width;
	}

	/**
	 * @brief Set the vertical half-height of the bounding box.
	 *
	 * This function sets the vertical half-height `half_height`, which defines half the vertical extent
	 * of the bounding box close to the camera's projection center, corresponding to the vertical extent of the
	 * frustum.
	 *
	 * @param half_height The vertical half-height, representing half the vertical extent of the bounding box.
	 */
	void set_bb_offset_X0_z(double half_height) {
		_bb_offset_X0_z = half_height;
	}

	/**
	 * @brief Calculate the camera's view angle from the principal distance and pixel size as well as the image dimensions.
	 */
	//void set_view_angles(double ck, double pixSize, int columns, int rows);
	void set_view_angles(const CameraIntrinsics& intr);

	// Getter functions for the half-width and half-height of the far plane in camera coordinates, including offsets for uncertainty (bounding box), replace xMax / -xMin and zMax / -zMin
	double get_halfWidth()  const { return _halfW; }   // Fernebene, Kamerasystem, inkl. Offset (ersetzt xMax / -xMin)
	double get_halfHeight() const { return _halfH; }   // Fernebene, Kamerasystem, inkl. Offset (ersetzt zMax / -zMin)

	double get_xmin_World()const { return _xMin_world; }
	double get_ymin_World()const { return _yMin_world; }
	double get_zmin_World()const { return _zMin_world; }
	double get_xmax_World()const { return _xMax_world; }
	double get_ymax_World()const { return _yMax_world; }
	double get_zmax_World()const { return _zMax_world; }
	
	double get_dist() const { return _d; }
	double get_Correction_backward()const { return _bb_offset_X0_xy / _tH; }
	

private:

	// logger + TAG
	LogFile* _logFilePrinter;
	const std::string TAG = "View Frustum:\t";

	// Frustum extents in world coordinates
	double _xMin_world, _xMax_world;
	double _yMin_world, _yMax_world;
	double _zMin_world, _zMax_world;

	// Frustum parameters
	double _bb_offset_X0_xy;// Radius (half-width) in local camera system
	double _bb_offset_X0_z; // Half-height in local camera system
	double _d;				// Distance from the camera
	double _tV, _tH;		// Tangent of vertical and horizontal field of view angles to calculate frustum from bounding box
	double _halfW, _halfH;  // half -width and half-height of the far plane in camera coordinates, including offsets for uncertainty (bounding box)
							//Halbachsen der Fernebene inkl. Lageunsicherheit (Kamerasystem)
};

#endif /* BOUNDINGBOX_H */

