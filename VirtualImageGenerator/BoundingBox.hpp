#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <sstream>
#include "LogFilePrinter.h"

#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

/* sets the visible Space.
The Pointfile will be searched for Points inside the bounding box, which is descriped by the space.
You can set the camera position with x0, y0 and z0.
The camera Orientation is descriped by azimuth.
And 3rd dimesnion of the bounding box will set with distance. This defines the distance from points to the camera.
OPTIONAL: dh and r.
dh set the accuracy for the height measurment. This defines the height from the bounding box at the camera to 2*dh.
r sets the accuracy from the position measurement (GNSS) and defines the width of the bounding Box at the camera to 2*r.
H,V descripes the View-angle in the horizontal and vertical plane. ( both must be in degrees !!)
CAUTION:
azimuth must be in degrees and is an angle from the y-axis to the camera view-axis.
*/
class BoundingBox
{
public:
	BoundingBox(LogFile* logfile);
	~BoundingBox();

	/* For the Bounding Box.
	This change the camera position (X0), orientation and depth from the bounding box  and calc the new bounding box.
	Its similar to:
	setX0(x0,y0,z0);
	setDist(distance);
	setAzi(azimuth);
	For more detail, look at the documentation from this methods.
	CAUTION: azimuth must be in degrees!! ( see setAzi() )
	*/
	void setSpace(double x0, double y0, double z0, float azimuth, float roll, float pitch, float distance);

	/* For the Bounding Box.
	This sets the camera position (X0) and calc the new bounding box, if recalcBB is true.
	The new bounding box will be translate by x0, y0 and z0.
	*/
	void set_X0_Cam_World(double x0, double y0, double z0);

	/* For the Bounding Box.
	This sets the camera orientation (azimuth) and calc the new bounding box, if recalcBB is true.
	The new bounding box will be rotate by azimuth.
	CAUTION: azimuth must be in degrees!! And descripe an angle from the y axis to the view axis of the camera.
	*/
	void setAngles(float azimuth, float roll, float pitch);

	/* For the Bounding Box.
	This sets the new depth of the bounding box and calc the new one, if recalcBB is true.
	The new bounding Box will have the depth of distance.
	So that all points with a distance to the camera < "distance" will be inside the bounding box.
	*/
	void setDist(float distance);

	////////////////////////////
	// accuracy paras

	/*This set the accuracy from the GNSS Sensor.
	The width of the bounding box at the camera will be 2*accGPS.
	After this method the bounding Box isn't recalculated. So use this method before
	setting X0, Azimuth or distance!*/
	void set_r(float accGPS) { r = accGPS; }

	/*Like set_r just for the height.
	The height of the bounding box at the camera will be 2*dh.
	The Bounding Box isn't recalculated here. Set this Parameter before setting X0, azimuth or distance.
	*/
	void set_dh(float h) { dh = h; }

	/*
	This sets the view Angle in the horizontal and vertical plane (must be in degrees!!)
	This dosn't recalc the boundingbox, so you must call this function before changing x0,dist or azimuth.
	If V isn't set (0.0f) it gets the same value as H.
	Values >= 60 degrees or <= 0 degrees are senseless or generate a to big image Plane. In this case the angles are not changed.
	*/
	void setViewAngle(float H, float V = 0.0f);


	// get Rz ( this stores the current orientation from the bounding Box
	float* _Rz() const { return Rz; }
	//float* _Ry() const { return Ry; }
	//float* _Rx() const { return Rx; }
	//float* _Rxyz() const { return Rxyz; }
	void set_Rz (float* _Rz) {
		Rz = _Rz;
	}

	// get X0 (stores the current position from the camera)
	double* get_X0_Cam_World()const { return X0_Cam_World; }

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

	float get_dist() { return d; }

	/*Return r/tan(H) this correction should be use to project all points with a bigger distance to the camera.
	Its usefull, because some points can be very very near, and this will result in a unaccesable image Plane.
	This returns only the correction in the Horizontal Plane, if H != V there will be different Corrections.
	But we use only this one, because in vertical plane are not so much points. */
	float get_Correction_backward()const { return r / tH; }

	void calcBoundingBox();

	void calc_rotationMatrix_xyz(float* _Rxyz); // in case of no rotation matrix from json, calc!


private:

	LogFile* logFilePrinter;

	const std::string TAG = "BoundingBox:\t";

	/*
	borders that determine the visible space.
	Only points inside this space will be loaded.
	*/
	double xMin_world, xMax_world, yMin_world, yMax_world, zMin_world, zMax_world;

	/*The coordinates of the edges from the bounding box in it's sytsem.
	These are used f.e. to calc the image plane.*/
	float xMin, yMin, zMin, xMax, yMax, zMax;

	/*recalc the bounding box with current values of
	X0,
	azimuth ( stored in Rz),
	d,
	dh and
	r.
	*/
	
	void calc_rotationMatrix_min_z(); // define rotation matrix -Rz

	/* Rotationsmatrix um die Z-Achse (für die Berechnung der Bounding Box). Berechnet aus dem negativen Azimut:
	Rz =		cos(azi)	sin(azi)	0
				-sin(az)	cos(azi)	0
				0			0			1

	mit Rz =	R0			R1			R2
				R3			R4			R5
				R6			R7			R8*/
	float* Rz;

	/* Rotationsmatrix um die Z-Achse (für die Berechnung der Bounding Box).
	Ry =		cos(roll)	0			sin(roll)
				0			1			0
				-sin(roll)	0			cos(roll)
	*/
	//float* Ry;

	//Drehung um y-Achse
	/*Ry[0] = Cy; Ry[1] = 0.0; Ry[2] = Sy;
	Ry[3] = 0.0; Ry[4] = 1.0; Ry[5] = 0.0;
	Ry[6] = -Sy; Ry[7] = 0.0; Ry[8] = Cy;
	*/


	//Drehung um x-Achse
	/*Rx[0] = 1.0; Rx[1] = 0.0; Rx[2] = 0.0;
	Rx[3] = 0.0; Rx[4] = Cx; Rx[5] = -Sx;
	Rx[6] = 0.0; Rx[7] = Sx; Rx[8] = Cx;
	*/

	/* Rotationsmatrix um die Y-Achse (für die Berechnung der Bounding Box).
	Rx =		1			0			0
				0			cos(pitch)	-sin(pitch)
				0			sin(pitch)	cos(pitch)

	*/

	//float* Rx;

	
	// Tiefe (3. Dimension der Bounding Box)
	float d;

	// Position der Kamera in Welt koordinaten (GNSS Messung)
	double* X0_Cam_World;

	// accuracy Parameter
	float r, dh;

	// view angle in horizontal and vertical plane (save as tan(A) )
	float tH, tV;

	float Cz, Sz, Cy, Sy, Cx, Sx; // cos & sin derivations for correpsonding angles arround x,y,z axis

};



#endif /* BOUNDINGBOX_H */

