#include "BoundingBox.hpp"



BoundingBox::BoundingBox(LogFile* logfile) {

	logFilePrinter = logfile; // ptr for logging
	logFilePrinter->append("");
	logFilePrinter->append(TAG + "Initialization of View Frustum"); 

	// Initialize Frustum borders locally
	xMin = yMin = zMin = 0.0;
	xMax = yMax = zMax = 0.0;
	d = 0.0;

	// Initialize rotation and Cos/Sin values to safe defaults
	Cx = Cy = Cz = 1.0; // cos(0°) = 1
	Sx = Sy = Sz = 0.0; // sin(0°) = 0

	r = 20.0; // frustum expansion - lateral
	dh = 25.0; // frustum expansion - height
	tV = tH = 0.57735; // view angles, vertical and horizontal, init with tan(30°)

	// Initialize Frustum borders in world space
	xMax_world = yMax_world = zMax_world = 0.0;
	xMin_world = yMin_world = zMin_world = 0.0;

	// Initialize camera position and Rz (rotation matrix)
	X0_Cam_World = new double[3];
	X0_Cam_World[0] = 0.0;
	X0_Cam_World[1] = 0.0;
	X0_Cam_World[2] = 0.0;
	
	// Initialize rotation matrix Rzxy (identity matrix by default), row-major
	Rzxy = new double[9];
	Rzxy[0] = 1.0; Rzxy[1] = 0.0; Rzxy[2] = 0.0; 
	Rzxy[3] = 0.0; Rzxy[4] = 1.0; Rzxy[5] = 0.0;
	Rzxy[6] = 0.0; Rzxy[7] = 0.0; Rzxy[8] = 1.0;
}


BoundingBox::~BoundingBox()
{
	if (X0_Cam_World != nullptr){
		delete[] X0_Cam_World;
		X0_Cam_World = nullptr;
	}
	if (Rzxy != nullptr){
		delete[] Rzxy;
		Rzxy = nullptr;
	}	
}


void BoundingBox::set_view_angles(double H, double V){
	if (H <= 0.0f || H >= 60.0f) return;
	tH = tan(H* M_PI / 180.0f);
	if (V <= 0.0f || V >= 60.0f) tV = tH;
	else tV = tan(V* M_PI / 180.0f);
}


void BoundingBox::set_X0_Cam_World(double x0, double y0, double z0){

	X0_Cam_World[0] = x0;
	X0_Cam_World[1] = y0;
	X0_Cam_World[2] = z0;
	logFilePrinter->append(TAG + "set X0 (XYZ) [m]: " + std::to_string(X0_Cam_World[0]) + "," + std::to_string(X0_Cam_World[1]) + "," + std::to_string(X0_Cam_World[2]));
}


void BoundingBox::calculate_rotation_matrix_rzxy(double _azimuth, double _roll, double _pitch){
	// All column-major!

	Cz = cos(_azimuth * M_PI / 180.0);
	Sz = sin(_azimuth *M_PI / 180.0);

	Cy = cos(_roll * M_PI / 180.0);
	Sy = sin(_roll * M_PI / 180.0);

	Cx = cos(_pitch * M_PI / 180.0);
	Sx = sin(_pitch * M_PI / 180.0);

	// Rzxy rotation order (ZXY rotation)
	Rzxy[0] = Cz * Cy - Sz * Sx * Sy;		Rzxy[3] = -Sz * Cx;		Rzxy[6] = Cz * Sy + Sz * Sx * Cy;
	Rzxy[1] = Sz * Cy + Cz * Sx * Sy;		Rzxy[4] = Cz * Cx;		Rzxy[7] = Sz * Sy - Cz * Sx * Cy;
	Rzxy[2] = -Cx * Sy;						Rzxy[5] = Sx;			Rzxy[8] = Cx * Cy;
}


void BoundingBox::calculate_view_frustum(){

	logFilePrinter->append(TAG + "Calculate View Frustum");
	
	// Simplest case: zMin and zMax grow with d*tan(V)
	zMin = -dh - d*tV;
	zMax = dh + d*tV;

	// Calculate xmin, xmax, ymin, ymax using the 4 points of the frustum box in the vertical plane (orthodirectional to optical axis)
	xMin = -r - d*tH;
	xMax = -xMin; // equivalent to r + d*tH
	yMin = 0;
	yMax = d;
	
	// Calculate rotated points of the view frustum
	double P1[2] = {-r * Rzxy[0], -r * Rzxy[1]};  // Rz * (-r,0,0)
	double P2[2] = {r * Rzxy[0], r * Rzxy[1]}; // Rz * (r,0,0)
	double P3[2] = { xMin * Rzxy[0] + d * Rzxy[3], xMin * Rzxy[1] + d * Rzxy[4] }; // Rz * (xMin,d,0)
	double P4[2] = { xMax * Rzxy[0] + d * Rzxy[3], xMax * Rzxy[1] + d * Rzxy[4] }; // Rz * (xMax,d,0)


	// Find the extrema for x and y 
	xMin_world = fmin(fmin(P1[0], P2[0]), fmin(P3[0], P4[0]));
	xMax_world = fmax(fmax(P1[0], P2[0]), fmax(P3[0], P4[0]));
	yMin_world = fmin(fmin(P1[1], P2[1]), fmin(P3[1], P4[1]));
	yMax_world = fmax(fmax(P1[1], P2[1]), fmax(P3[1], P4[1]));

	// Translate to world coordinates
	xMin_world += X0_Cam_World[0]; 
	xMax_world += X0_Cam_World[0];
	yMin_world += X0_Cam_World[1]; 
	yMax_world += X0_Cam_World[1];
	zMin_world = zMin + X0_Cam_World[2];
	zMax_world = zMax + X0_Cam_World[2];

	// Log frustum information
	logFilePrinter->append(TAG	+ "Defined View Frustum (camera system): ");
	logFilePrinter->append(TAG	+ "P1: (" + std::to_string(P1[0]) + "," + std::to_string(P1[1]) 
								+ "), P2: (" + std::to_string(P2[0]) + "," + std::to_string(P2[1]) 
								+ "), P3: (" + std::to_string(P3[0]) + "," + std::to_string(P3[1]) 
								+ "), P4: (" + std::to_string(P4[0]) + "," + std::to_string(P4[1]) + ")");
	logFilePrinter->append(TAG + "Defined View Frusum (world system): ");
	logFilePrinter->append(TAG + "(xmin,ymin,zmin): (" + std::to_string(xMin_world) + "," + std::to_string(yMin_world) + "," + std::to_string(zMin_world) + ")");
	logFilePrinter->append(TAG + "(xmax,ymax,zmax): (" + std::to_string(xMax_world) + "," + std::to_string(yMax_world) + "," + std::to_string(zMax_world) + ")");

	//logFilePrinter->append(TAG + "Rz:");
	//logFilePrinter->append("\t\t" + std::to_string(Rzxy[0]) + " " + std::to_string(Rzxy[3]) + " " + std::to_string(Rzxy[6]), 4);
	//logFilePrinter->append("\t\t" + std::to_string(Rzxy[1]) + " " + std::to_string(Rzxy[4]) + " " + std::to_string(Rzxy[7]), 4);
	//logFilePrinter->append("\t\t" + std::to_string(Rzxy[2]) + " " + std::to_string(Rzxy[5]) + " " + std::to_string(Rzxy[8]), 4);
	//logFilePrinter->append("");
}