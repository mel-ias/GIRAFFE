#include "BoundingBox.hpp"

BoundingBox::BoundingBox(LogFile* logfile) {

	_logFilePrinter = logfile; // ptr for logging
	_logFilePrinter->append("");
	_logFilePrinter->append(TAG + "Initialization of View Frustum"); 

	// Initialize Frustum borders locally
	_xMin = _yMin = _zMin = 0.0;
	_xMax = _yMax = _zMax = 0.0;
	_d = 0.0;

	_bb_offset_X0_xy = 20.0; // frustum expansion - lateral
	_bb_offset_X0_z = 25.0; // frustum expansion - height
	_tV = _tH = 0.57735; // view angles, vertical and horizontal, init with tan(30°)

	// Initialize Frustum borders in world space
	_xMax_world = _yMax_world = _zMax_world = 0.0;
	_xMin_world = _yMin_world = _zMin_world = 0.0;

	// Initialize camera position
	_X0_cam_world = new double[3];
	_X0_cam_world[0] = 0.0;
	_X0_cam_world[1] = 0.0;
	_X0_cam_world[2] = 0.0;
	
	// Initialize rotation matrix Rzxy (identity matrix by default), row-major
	_Rzxy = new double[9];
	_Rzxy[0] = 1.0; _Rzxy[1] = 0.0; _Rzxy[2] = 0.0; 
	_Rzxy[3] = 0.0; _Rzxy[4] = 1.0; _Rzxy[5] = 0.0;
	_Rzxy[6] = 0.0; _Rzxy[7] = 0.0; _Rzxy[8] = 1.0;
}


BoundingBox::~BoundingBox()
{
	if (_X0_cam_world != nullptr){
		delete[] _X0_cam_world;
		_X0_cam_world = nullptr;
	}
	if (_Rzxy != nullptr){
		delete[] _Rzxy;
		_Rzxy = nullptr;
	}	
}


void BoundingBox::set_view_angles(double H, double V){
	if (H <= 0.0f || H >= 60.0f) return;
	_tH = tan(H* M_PI / 180.0f);
	if (V <= 0.0f || V >= 60.0f) _tV = _tH;
	else _tV = tan(V* M_PI / 180.0f);
}


void BoundingBox::set_X0_Cam_World(double X0_x, double X0_y, double X0_z){

	_X0_cam_world[0] = X0_x;
	_X0_cam_world[1] = X0_y;
	_X0_cam_world[2] = X0_z;
	_logFilePrinter->append(TAG + "set X0 (XYZ) [m]: " + std::to_string(_X0_cam_world[0]) + "," + std::to_string(_X0_cam_world[1]) + "," + std::to_string(_X0_cam_world[2]));
}


void BoundingBox::calculate_rotation_matrix_rzxy(double _azimuth, double _roll, double _pitch){

	// Trigonometric values for rotation angles
	double Cz = cos(_azimuth * M_PI / 180.0);
	double Sz = sin(_azimuth *M_PI / 180.0);

	double Cy = cos(_roll * M_PI / 180.0);
	double Sy = sin(_roll * M_PI / 180.0);

	double Cx = cos(_pitch * M_PI / 180.0);
	double Sx = sin(_pitch * M_PI / 180.0);

	// all column-major!
	// Rzxy rotation order (ZXY rotation)
	_Rzxy[0] = Cz * Cy - Sz * Sx * Sy;		_Rzxy[3] = -Sz * Cx;	_Rzxy[6] = Cz * Sy + Sz * Sx * Cy;
	_Rzxy[1] = Sz * Cy + Cz * Sx * Sy;		_Rzxy[4] = Cz * Cx;		_Rzxy[7] = Sz * Sy - Cz * Sx * Cy;
	_Rzxy[2] = -Cx * Sy;					_Rzxy[5] = Sx;			_Rzxy[8] = Cx * Cy;
}


void BoundingBox::calculate_view_frustum(){

	_logFilePrinter->append(TAG + "Calculate View Frustum");
	
	// Simplest case: zMin and zMax grow with d*tan(V)
	_zMin = -(_bb_offset_X0_z) - (_d * _tV);
	_zMax = _bb_offset_X0_z + (_d * _tV);

	// Calculate xmin, xmax, ymin, ymax using the 4 points of the frustum in the vertical plane (orthodirectional to optical axis)
	_xMin = -(_bb_offset_X0_xy) - (_d * _tH);
	_xMax = -(_xMin); // equivalent to r + d*tH
	_yMin = 0;
	_yMax = _d;
	
	// Calculate rotated points of the view frustum
	double P1[2] = {-(_bb_offset_X0_xy) * _Rzxy[0], -(_bb_offset_X0_xy) * _Rzxy[1]};  // Rzxy * (-bb_offset_X0 (xy),0,0)
	double P2[2] = {_bb_offset_X0_xy * _Rzxy[0], _bb_offset_X0_xy * _Rzxy[1]}; // Rzxy * (bb_offset_X0 (xy),0,0)
	double P3[2] = { _xMin * _Rzxy[0] + _d * _Rzxy[3], _xMin * _Rzxy[1] + _d * _Rzxy[4] }; // Rzxy * (xMin,d,0)
	double P4[2] = { _xMax * _Rzxy[0] + _d * _Rzxy[3], _xMax * _Rzxy[1] + _d * _Rzxy[4] }; // Rzxy * (xMax,d,0)

	// Find the extrema for x and y 
	_xMin_world = fmin(fmin(P1[0], P2[0]), fmin(P3[0], P4[0]));
	_xMax_world = fmax(fmax(P1[0], P2[0]), fmax(P3[0], P4[0]));
	_yMin_world = fmin(fmin(P1[1], P2[1]), fmin(P3[1], P4[1]));
	_yMax_world = fmax(fmax(P1[1], P2[1]), fmax(P3[1], P4[1]));

	// Translate to world coordinates
	_xMin_world += _X0_cam_world[0]; 
	_xMax_world += _X0_cam_world[0];
	_yMin_world += _X0_cam_world[1]; 
	_yMax_world += _X0_cam_world[1];
	_zMin_world = _zMin + _X0_cam_world[2];
	_zMax_world = _zMax + _X0_cam_world[2];

	// Log frustum information
	_logFilePrinter->append(TAG	+ "Defined View Frustum (camera system): ");
	_logFilePrinter->append(TAG	+ "P1: (" + std::to_string(P1[0]) + "," + std::to_string(P1[1]) 
								+ "), P2: (" + std::to_string(P2[0]) + "," + std::to_string(P2[1]) 
								+ "), P3: (" + std::to_string(P3[0]) + "," + std::to_string(P3[1]) 
								+ "), P4: (" + std::to_string(P4[0]) + "," + std::to_string(P4[1]) + ")");
	_logFilePrinter->append(TAG + "Defined View Frusum (world system): ");
	_logFilePrinter->append(TAG + "(xmin,ymin,zmin): (" + std::to_string(_xMin_world) + "," + std::to_string(_yMin_world) + "," + std::to_string(_zMin_world) + ")");
	_logFilePrinter->append(TAG + "(xmax,ymax,zmax): (" + std::to_string(_xMax_world) + "," + std::to_string(_yMax_world) + "," + std::to_string(_zMax_world) + ")");
}

