#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <sstream>
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
	~BoundingBox();

	/**
	 * @brief Set the camera's orientation based on azimuth, roll, and pitch angles.
	 *
	 * This function computes the rotation matrix (Rzxy) using the provided azimuth, roll,
	 * and pitch angles. The rotation follows a ZXY rotation order:
	 * 1. Azimuth (z-axis)
	 * 2. Pitch (x-axis)
	 * 3. Roll (y-axis)
	 * http://www.songho.ca/opengl/gl_anglestoaxes.html
	 *
	 * The rotation matrix is column-major, and the function updates the class member Rz.
	 *
	 * @param _azimuth The azimuth angle (in degrees).
	 * @param _roll The roll angle (in degrees).
	 * @param _pitch The pitch angle (in degrees).
	 */
	void calculate_rotation_matrix_rzxy(float azimuth, float roll, float pitch);


	/**
	 * @brief Recalculate the view frustum based on the current camera parameters.
	 *
	 * This function recalculates the view frustum dimensions using the camera's position,
	 * azimuth, distance, and other geometric properties. It updates the view frustum's
	 * world and local coordinates, as well as the min/max values in the x, y, and z directions.
	 *
	 * The view frustum is computed in the local camera coordinate system and then transformed
	 * to the global world space using the rotation matrix Rz and translation by X0_Cam_World.
	 */
	void calculate_view_frustum();

	/**
	 * @brief Define the camera's world position for view frustum calculation.
	 *
	 * This function sets the camera position in the world coordinates.
	 * The view frustum is then translated by the provided position.
	 *
	 * @param x0 X coordinate of the camera in world coordinates.
	 * @param y0 Y coordinate of the camera in world coordinates.
	 * @param z0 Z coordinate of the camera in world coordinates.
	 */
	void set_X0_Cam_World(double x0, double y0, double z0);
	
	/**
	 * @brief Set the depth of the view frustum.
	 *
	 * This function sets the depth of the view frustum to the given distance from the camera.
	 * All points within the specified distance from the camera will be inside the view frustum.
	 *
	 * @param distance The distance from the camera, defining the new depth of the view frustum.
	 */
	void set_frustum_depth(float distance) {
		d = distance;
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
	void set_r(float half_width) {
		r = half_width;
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
	void set_dh(float half_height) {
		dh = half_height;
	}

	/**
	 * @brief Set the camera's view angle.
	 *
	 * This function sets the horizontal and vertical angles of view for the view frustum.
	 *
	 * @param H Horizontal angle in degrees (must be between 0 and 60 degrees).
	 * @param V Vertical angle in degrees (must be between 0 and 60 degrees, defaulted to H if invalid).
	 */
	void set_view_angles(float H, float V = 0.0f);

	/**
	 * @brief Set the 3x3 rotation matrix in ZXY order.
	 *
	 * This function sets the rotation matrix `Rzxy`, which defines the transformation for the bounding box.
	 * The rotation matrix is applied using the ZXY order of rotations: first around the Z-axis (azimuth),
	 * followed by the X-axis (pitch), and finally the Y-axis (roll).
	 *
	 * The matrix is expected to be in column-major order, and the transformation is used to rotate
	 * the bounding box from the world coordinates to the local camera coordinates.
	 *
	 * @param _Rz A pointer to a 3x3 rotation matrix in column-major order, representing ZXY rotations.
	 */
	void set_Rzxy(float* Rz) { Rzxy = Rz; }


	float get_xMin()const { return xMin; }
	float get_xMax()const { return xMax; }
	float get_yMin()const { return yMin; }
	float get_yMax()const { return yMax; }
	float get_zMax()const { return zMax; }
	float get_zMin()const { return zMin; }

	double get_xmin_World()const { return xMin_world; }
	double get_ymin_World()const { return yMin_world; }
	double get_zmin_World()const { return zMin_world; }
	double get_xmax_World()const { return xMax_world; }
	double get_ymax_World()const { return yMax_world; }
	double get_zmax_World()const { return zMax_world; }
	
	float get_dist() const { return d; }
	float get_Correction_backward()const { return r / tH; }
	
	float* get_Rzxy() const { return Rzxy; }
	double* get_X0_Cam_World()const { return X0_Cam_World; }

private:

	// logger + TAG
	LogFile* logFilePrinter;
	const std::string TAG = "View Frustum:\t";

	// Rotation matrix in ZXY order (column-major)
	float* Rzxy;
	
	// Translation vector (camera origin in world coordinates)
	double* X0_Cam_World;

	// Frustum extents in world coordinates
	double xMin_world, xMax_world;
	double yMin_world, yMax_world;
	double zMin_world, zMax_world;

	// Frustum parameters
	float r;      // Radius (half-width) in local camera system
	float dh;     // Half-height in local camera system
	float d;      // Distance from the camera
	float tV, tH; // Tangent of vertical and horizontal field of view angles to calculate frustum from bounding box
	float zMin, zMax; // Min and max z values in local camera system
	float xMin, xMax; // Min and max x values in local camera system
	float yMin, yMax; // Min and max y values in local camera system

	// Trigonometric values for rotation angles
	float Cz, Sz; // Cosine and sine of azimuth
	float Cx, Sx; // Cosine and sine of pitch
	float Cy, Sy; // Cosine and sine of roll
};

#endif /* BOUNDINGBOX_H */

