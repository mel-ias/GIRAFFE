#include "BoundingBox.hpp"

BoundingBox::BoundingBox(LogFile* logfile) {

	_logFilePrinter = logfile; // ptr for logging
	_logFilePrinter->append("");
	_logFilePrinter->append(TAG + "Initialization of View Frustum");

	_halfW = _halfH = 0.0;
	_d = 0.0;

	_bb_offset_X0_xy = 20.0; // frustum expansion - lateral
	_bb_offset_X0_z = 25.0; // frustum expansion - height
	_tV = _tH = 0.57735; // view angles, vertical and horizontal, init with tan(30°)

	// Initialize Frustum borders in world space
	_xMax_world = _yMax_world = _zMax_world = 0.0;
	_xMin_world = _yMin_world = _zMin_world = 0.0;
}


// Eingabe: Kamera/Pixelgeometrie
//void BoundingBox::set_view_angles(double ck, double pixSize, int columns, int rows) {
//	_tH = (columns * pixSize / 2.0) / ck;
//	_tV = (rows * pixSize / 2.0) / ck;
//	_logFilePrinter->append(TAG + "set view angle tH: " + std::to_string(_tH) + ", tV: " + std::to_string(_tV));
//}
void BoundingBox::set_view_angles(const CameraIntrinsics& intr) {
	_tH = intr.tanHalfFovH();
	_tV = intr.tanHalfFovV();
	_logFilePrinter->append(TAG + "set view angle tH: " + std::to_string(_tH) + ", tV: " + std::to_string(_tV));
}


void BoundingBox::update(const CameraPose& pose) {

	_logFilePrinter->append(TAG + "Update View Frustum");

	// Kamerasystem (OpenCV: x rechts, y unten, z Blickrichtung): Halbachsen der Fernebene inkl. Lageunsicherheit
	_halfW = _bb_offset_X0_xy + _d * _tH;
	_halfH = _bb_offset_X0_z + _d * _tV;

	// Weltsystem: Spitze + vier Eckpunkte der Fernebene, achsenparallele Hülle
	const double w = _d * _tH, h = _d * _tV;
	const cv::Vec3d corners[5] = { {0, 0, 0}, {-w, -h, _d}, {w, -h, _d}, {-w, h, _d}, {w, h, _d} };
	const cv::Matx33d R_cw = pose.R_wc.t();

	cv::Vec3d lo(DBL_MAX, DBL_MAX, DBL_MAX), hi(-DBL_MAX, -DBL_MAX, -DBL_MAX);
	for (const auto& p : corners) {
		const cv::Vec3d X = pose.C + R_cw * p;
		for (int k = 0; k < 3; ++k) {
			lo[k] = std::min(lo[k], X[k]);
			hi[k] = std::max(hi[k], X[k]);
		}
	}

	// Lageunsicherheit (loc_accuracy) im Weltsystem aufschlagen
	_xMin_world = lo[0] - _bb_offset_X0_xy;  _xMax_world = hi[0] + _bb_offset_X0_xy;
	_yMin_world = lo[1] - _bb_offset_X0_xy;  _yMax_world = hi[1] + _bb_offset_X0_xy;
	_zMin_world = lo[2] - _bb_offset_X0_z;   _zMax_world = hi[2] + _bb_offset_X0_z;

	// Log frustum information
	_logFilePrinter->append(TAG + "Defined View Frustum (world system): ");
	_logFilePrinter->append(TAG + "(xmin,ymin,zmin): (" + std::to_string(_xMin_world) + "," + std::to_string(_yMin_world) + "," + std::to_string(_zMin_world) + ")");
	_logFilePrinter->append(TAG + "(xmax,ymax,zmax): (" + std::to_string(_xMax_world) + "," + std::to_string(_yMax_world) + "," + std::to_string(_zMax_world) + ")");
}