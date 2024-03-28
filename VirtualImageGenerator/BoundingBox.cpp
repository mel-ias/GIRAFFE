#include "BoundingBox.hpp"
#include<iostream>
#define _USE_MATH_DEFINES
#include<math.h>


BoundingBox::BoundingBox(LogFile* logfile) {

	logFilePrinter = logfile; // ptr for logging
	logFilePrinter->append(""); // return one line
	logFilePrinter->append(TAG + "---- initialisation bounding box ----"); // return one line

	// init Border
	xMin = yMin = zMin = 0.0f;
	xMax = yMax = zMax = 0.0f;
	d = 0.0f;

	r = 20.0f; // GPS Rauschen 20 m
	dh = 25.0f; // erzeugt 50m in der höhen komponente des Bildes
	tV = tH = 0.57735f; // tan(30)

	// init world borlder
	xMax_world = yMax_world = zMax_world = 0.0f;
	xMin_world = yMin_world = zMin_world = 0.0f;

	// init Position and Rz
	X0_Cam_World = new double[3];
	
	// note! row major!
	Rz = new float[9];
	Rz[0] = 1.0f; Rz[1] = 0.0f; Rz[2] = 0.0f; 
	Rz[3] = 0.0f; Rz[4] = 1.0f; Rz[5] = 0.0f;
	Rz[6] = 0.0f; Rz[7] = 0.0f; Rz[8] = 1.0f;


}


BoundingBox::~BoundingBox()
{

	if (X0_Cam_World != nullptr){
		delete[] X0_Cam_World;
		X0_Cam_World = nullptr;
	}
	if (Rz != nullptr){
		delete[] Rz;
		Rz = nullptr;
	}

	
}


void BoundingBox::setViewAngle(float H, float V){
	if (H <= 0.0f || H >= 60.0f) return;
	tH = static_cast<float>(tan(H* M_PI / 180.0f));
	if (V <= 0.0f || V >= 60.0f) tV = tH;
	else tV = static_cast<float>(tan(V* M_PI / 180.0f));
}

/* For the Bounding Box.
This sets the camera position (X0) and calc the new bounding box, if recalcBB is true.
The new bounding box will be translate by x0, y0 and z0.
*/
void BoundingBox::set_X0_Cam_World(double x0, double y0, double z0){

	X0_Cam_World[0] = x0;
	logFilePrinter->append(TAG + "BBox, set x0: " + std::to_string(X0_Cam_World[0]));
	X0_Cam_World[1] = y0;
	logFilePrinter->append(TAG + "BBox, set y0: " + std::to_string(X0_Cam_World[1]));
	X0_Cam_World[2] = z0;
	logFilePrinter->append(TAG + "BBox, set z0: " + std::to_string(X0_Cam_World[2]));
	// we have translate the bounding Box
}


/* For the Bounding Box.
This sets the new depth of the bounding box and calc the new one, if recalcBB is true.
The new bounding Box will have the depth of distance.
So that all points with a distance to the camera < "distance" will be inside the bounding box.
*/
void BoundingBox::setDist(float distance){
	d = distance;
	// we have change one dimension from the Bounding Box
}


/* For the Bounding Box.
This sets the camera orientation (azimuth) and calc the new bounding box, if recalcBB is true.
The new bounding box will be rotate by azimuth.
CAUTION: azimuth must be in degrees!! And descripe an angle from the y axis to the view axis from the camera.
*/

//http://www.songho.ca/opengl/gl_anglestoaxes.html

// azimuth dreht um z achse, roll dreht um y Achse, pitch dreht um x Achse
void BoundingBox::setAngles(float _azimuth, float _roll, float _pitch){

	// all column major!

	Cz = static_cast<float>(cos(_azimuth * M_PI / 180.0f));
	Sz = static_cast<float>(sin(_azimuth *M_PI / 180.0f));

	Cy = static_cast<float>(cos(_roll * M_PI / 180.0f));
	Sy = static_cast<float>(sin(_roll * M_PI / 180.0f));

	Cx = static_cast<float>(cos(_pitch * M_PI / 180.0f));
	Sx = static_cast<float>(sin(_pitch * M_PI / 180.0f));

	calc_Rz(); // calc rotation matrix -Rz
	
}

void BoundingBox::calcRotM_XYZ(float* Rxyz) {

	// Rxyz, performs 3 rotations in order of Rz (azi), Ry (roll) then Rx (pitch).
	// determine left axis [x, pitch]	determine up axis [y, roll]		determine forward axis [z, Azimuth]	
	Rxyz[0] = Cy*Cz;				Rxyz[3] = -Cy*Sz;				Rxyz[6] = Sy;
	Rxyz[1] = Sx*Sy*Cz + Cx*Sz;		Rxyz[4] = -Sx*Sy*Sz + Cx*Cz;	Rxyz[7] = -Sx*Cy;
	Rxyz[2] = -Cx*Sy*Cz + Sx*Sz;	Rxyz[5] = Cx*Sy*Sz + Sx*Cz;		Rxyz[8] = Cx*Cy;
}

// here we will calculate Rz (rotation around z-axis (Azimuth)) clockwise
void BoundingBox::calc_Rz() {

	/* Rotationsmatrix um die Z-Achse (für die Berechnung der Bounding Box). Berechnet aus dem negativen Azimut:
	Rz =		cos(azi)	sin(azi)	0
				-sin(az)	cos(azi)	0
				0			0			1
	mit Rz =	R0			R1			R2
				R3			R4			R5
				R6			R7			R8*/

	// Rzyx rotation order, see https://www.songho.ca/opengl/gl_anglestoaxes.html for details
	Rz[0] = Cz * Cy;		Rz[3] = -Sz * Cx + Cz * Sy * Sx;		Rz[6] = Sz * Sx + Cz * Sy * Cx;
	Rz[1] = Sz * Cy;		Rz[4] = Cz * Cx + Sz * Sy * Sx;			Rz[7] = -Cz * Sx + Sz * Sy * Cx;
	Rz[2] = -Sy;			Rz[5] = Cy * Sx;						Rz[8] = Cy * Cx;



	// workes, consideres rotation direction (north to east to south to north)
	/*Rz[0] = Cz;		Rz[1] = -Sz;	Rz[2] = 0.0;
	Rz[3] = Sz;		Rz[4] = Cz;		Rz[5] = 0.0;
	Rz[6] = 0.0;	Rz[7] = 0.0;	Rz[8] = 1.0;*/

	//// correct from Richards descripzion, necessary to rotate azi like a compass works (north to east to south to north)
	//Rz[0] = Cz;	Rz[1] = Sz;		Rz[2] = 0.0;
	//Rz[3] = -Sz;	Rz[4] = Cz;		Rz[5] = 0.0;
	//Rz[6] = 0.0;	Rz[7] = 0.0;	Rz[8] = 1.0;
	 	 
	
	
		 
		 
	

}


/* For the Bounding Box.
This change the camera position, orientation and depth from the bounding box (X0) and calc the new bounding box.
Its similar to:
setX0(x0,y0,z0);
setDist(distance);
setAzi(azimuth);
For more detail, look at the documentation from this methods.
CAUTION: azimuth must be in degrees!! ( see setAzi() )
*/
void BoundingBox::setSpace(double x0, double y0, double z0, float azimuth, float roll, float pitch, float distance){
	set_X0_Cam_World(x0, y0, z0);
	setDist(distance);
	setAngles(azimuth, roll, pitch);
	calcBoundingBox();
}


/*recalc the bounding box with current values of
X0,
azimuth ( stored in Rz),
d,
dh, r , tan(V) and tan(H).
*/
void BoundingBox::calcBoundingBox(){

	logFilePrinter->append(TAG + "calc bounding box");
	// simpliest: h defines zMin and zMax and grows with d*tan(V)
	zMin = -dh - d*tV;
	zMax = dh + d*tV;

	// now xmin, xmax, ymin and ymax
	// for that we calc the 4 points of the bounding box in the horizontal plane

	// this are the edges of the bounding box in the lokal camera system 
	// to check global points, the global points will be transformed to the camerasystem (Rz*(x-x0))
	xMin = -r - d*tH;
	xMax = -xMin; //r + d*tH
	yMin = 0;
	yMax = d;
	
	
	// use Rz only
	// P1 and P2 are the border near to the camera
	// zeilenbasiert multiplizeren. nicht elementweise!
	// Rz*(-r,0,0) // mit erster Zeile für P1x und 2. für P1y multiplizieren
	float P1[2] = { 
		-r*Rz[0] + 0*Rz[3] + 0*Rz[6], 
		-r*Rz[1] + 0*Rz[4] + 0*Rz[7] }; 
	
	// Rz*(r,0,0)
	float P2[2] = { // mit erster Zeile für P2x und 2. für P2y multiplizieren
		r*Rz[0] + 0 * Rz[3] + 0 * Rz[6],
		r*Rz[1] + 0 * Rz[4] + 0 * Rz[7] };
	
	// P3 and P4 are the border far from the camera 
	// calculate as Rz*(xMin,d,0) 	
	float P3[2] = { 
		xMin*Rz[0] + d*Rz[3] + 0*Rz[6], 
		xMin*Rz[1] + d*Rz[4] + 0*Rz[7] };
	
	// calculate as Rz*(xMax,d,0) 	
	float P4[2] = { 
		xMax*Rz[0] + d*Rz[3] + 0*Rz[6], 
		xMax*Rz[1] + d*Rz[4] + 0*Rz[7] }; 
	
	

	// calc the world coordinates of these edges. (for point searching)
	// the extrema from x and y and z are the extrema from this 4 points

	// rotationsmatrix zxy! 
	xMin_world = fminf(fminf(P1[0], P2[0]), fminf(P3[0], P4[0]));
	xMax_world = fmaxf(fmaxf(P1[0], P2[0]), fmaxf(P3[0], P4[0]));

	yMin_world = fminf(fminf(P1[1], P2[1]), fminf(P3[1], P4[1]));
	yMax_world = fmaxf(fmaxf(P1[1], P2[1]), fmaxf(P3[1], P4[1]));

	// now add the translation to the boundingbox
	xMin_world += X0_Cam_World[0]; 
	xMax_world += X0_Cam_World[0];
	
	yMin_world += X0_Cam_World[1]; 
	yMax_world += X0_Cam_World[1];

	zMin_world = zMin + X0_Cam_World[2];
	zMax_world = zMax + X0_Cam_World[2];

	logFilePrinter->append(TAG + "defined bounding box (local camera system): ");
	logFilePrinter->append(TAG 
		+ "P1: (" + std::to_string(P1[0]) + "," + std::to_string(P1[1]) 
		+ "), P2: (" + std::to_string(P2[0]) + "," + std::to_string(P2[1]) 
		+ "), P3: (" + std::to_string(P3[0]) + "," + std::to_string(P3[1]) 
		+ "), P4: (" + std::to_string(P4[0]) + "," + std::to_string(P4[1]) + ")");
	logFilePrinter->append(TAG + "defined bounding box (global system): ");
	logFilePrinter->append(TAG + "(xmin,ymin,zmin): (" + std::to_string(xMin_world) + "," + std::to_string(yMin_world) + "," + std::to_string(zMin_world) + ")");
	logFilePrinter->append(TAG + "(xmax,ymax,zmax): (" + std::to_string(xMax_world) + "," + std::to_string(yMax_world) + "," + std::to_string(zMax_world) + ")");

	logFilePrinter->append(TAG + "Rz:");
	logFilePrinter->append("\t\t" + std::to_string(Rz[0]) + " " + std::to_string(Rz[3]) + " " + std::to_string(Rz[6]), 4);
	logFilePrinter->append("\t\t" + std::to_string(Rz[1]) + " " + std::to_string(Rz[4]) + " " + std::to_string(Rz[7]), 4);
	logFilePrinter->append("\t\t" + std::to_string(Rz[2]) + " " + std::to_string(Rz[5]) + " " + std::to_string(Rz[8]), 4);
	logFilePrinter->append("");
	
	/*if (logFilePrinter != nullptr)
		logFilePrinter->logfile_all << logfile.str() << std::endl;
	*/

}
