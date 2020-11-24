//#include "Modell.h"
//
//
//
//Modell::Modell(LogFile* _logfile, cv::Mat& _transfMat, cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs, int _imageWidth, int _imageHeight, double _pixelGr, std::vector<cv::Point2d>* _waterlinePoints) {
//	
//	logfile = _logfile;
//	logfile->append("");
//	logfile->append(TAG + "---- initialisation photogrammetric modell ----");
//
//	transformationsMatrix = _transfMat;
//	cameraMatrix = _cameraMatrix;
//	distCoeffs = _distCoeffs;
//	
//	imageWidth = _imageWidth;
//	imageHeight = _imageHeight;
//	pixelGr = _pixelGr;
//
//	waterlinePoints = _waterlinePoints;
//	waterlinePoints_projected = new std::vector<cv::Point3d>;
//
//
//	conversion();
//	initialisePointCloudProjection();
//
//
//
//
//}
//
//Modell::~Modell() {
//	if (waterlinePoints_projected != nullptr)
//		delete waterlinePoints_projected;
//}
//
//
//
//
//
//void Modell::conversion() {
//
//	
//
//	// 1. Innere Orientierung
//	double xh = imageWidth * pixelGr / 2 - cameraMatrix.at<double>(0,2) * pixelGr; // Umrechnung in mm-Abweichung von Ideal-Hauptpunkt
//	xh = xh * (-1);
//	double yh = imageHeight * pixelGr / 2 - cameraMatrix.at<double>(1,2) * pixelGr;
//	double fxfy = cameraMatrix.at<double>(0, 0);
//	double ck = fxfy * pixelGr;
//
//	trans_cameraMatrix = (cv::Mat_<double>(3, 3) << ck, 0, xh, 0, ck, yh, 0, 0, 1);
//
//	
//	//std::vector<double> data(trans_cameraMatrix.ptr(), trans_cameraMatrix.ptr() + 9);
//	//std::string stringmat(data.begin(), data.end());
//	//logfile->append(TAG + "trans_cam: " + stringmat);
//	
//	
//	// 2. Verzeichnung
//	double k1 = (distCoeffs.at<double>(0,0) / std::pow(ck, 1));
//	double k2 = (distCoeffs.at<double>(1,0) / std::pow(ck, 3)); 
//	double k3 = (distCoeffs.at<double>(4,0) / std::pow(ck, 5));
//
//	double p1 = (distCoeffs.at<double>(2,0) / fxfy * pixelGr);
//	double p2 = (distCoeffs.at<double>(3,0) / fxfy * pixelGr);
//
//
//	//	double b1 = (verzeichnungsKoeffizienten.getmParaC().getmC1() / fxfy * pixelGr);
//	//	double b2 = (verzeichnungsKoeffizienten.getmParaC().getmC2() / fxfy * pixelGr);
//
//	trans_distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
//
//	
//	// 3. Äußere Orientierung
//
//	// Rechne Transformationsmatrix um und verrechne Skalierung entsprechend TODO: Ergebnisse im Auge behalten!
//	// hole Rotationsmatrix ohne Berücksichtigung der Skalierung heraus und speichere diese zwischen
//	cv::Rect r(0, 0, 3, 3);
//	cv::Mat transformationsMatrix_copy = transformationsMatrix;
//
//	cv::Mat rotationsMatrix_CV = transformationsMatrix_copy(r); //shallowcopy
//
//
//	double scale_sx = std::sqrt(std::pow(rotationsMatrix_CV.at<double>(0, 0), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(1, 0), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(2, 0), 2));
//
//	double scale_sy = std::sqrt(std::pow(rotationsMatrix_CV.at<double>(0, 1), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(1, 1), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(2, 1), 2));
//
//	double scale_sz = std::sqrt(std::pow(rotationsMatrix_CV.at<double>(0, 2), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(1, 2), 2)
//		+ std::pow(rotationsMatrix_CV.at<double>(2, 2), 2));
//
//	// Berechne aus Länge der Spaltenvektoren der Rotationsmatrix die entsprechenden Skalierungswerte
//	// scale sollte dabei nahezu 1 sein um größere Probleme zu vermeiden ;)
//
//	cv::Scalar s = (scale_sx, scale_sy, scale_sz);
//
//	// beziehe Scale auf Rotationsmatrix
//	rotationsMatrix_CV.at<double>(0, 0) = rotationsMatrix_CV.at<double>(0, 0) / scale_sx;
//	rotationsMatrix_CV.at<double>(1, 0) = rotationsMatrix_CV.at<double>(1, 0) / scale_sx;
//	rotationsMatrix_CV.at<double>(2, 0) = rotationsMatrix_CV.at<double>(2, 0) / scale_sx;
//
//	rotationsMatrix_CV.at<double>(0, 1) = rotationsMatrix_CV.at<double>(0, 1) / scale_sy;
//	rotationsMatrix_CV.at<double>(1, 1) = rotationsMatrix_CV.at<double>(1, 1) / scale_sy;
//	rotationsMatrix_CV.at<double>(2, 1) = rotationsMatrix_CV.at<double>(2, 1) / scale_sy;
//
//	rotationsMatrix_CV.at<double>(0, 2) = rotationsMatrix_CV.at<double>(0, 2) / scale_sz;
//	rotationsMatrix_CV.at<double>(1, 2) = rotationsMatrix_CV.at<double>(1, 2) / scale_sz;
//	rotationsMatrix_CV.at<double>(2, 2) = rotationsMatrix_CV.at<double>(2, 2) / scale_sz;
//
//
//	cv::Mat trans = cv::Mat::zeros(3, 3, CV_64FC1); // Assuming no lens distortion
//	trans.at<double>(0, 0) = 1.0;
//	trans.at<double>(1, 1) = -1.0;
//	trans.at<double>(2, 2) = -1.0;
//
//
//	/***************** ANDROID ***********************/
//
//	cv::Mat negative_RotationMat_Cv = rotationsMatrix_CV * trans;
//	
//	/***************** END ANDROID *******************/
//
//	// Projektionszentrum bleibt unverändert aus transfMatrix
//	transformationsMatrix.copyTo(trans_transformationMatrix);
//
//	trans_transformationMatrix.at<double>(0, 0) = negative_RotationMat_Cv.at<double>(0, 0);
//	trans_transformationMatrix.at<double>(1, 0) = negative_RotationMat_Cv.at<double>(1, 0);
//	trans_transformationMatrix.at<double>(2, 0) = negative_RotationMat_Cv.at<double>(2, 0);
//	trans_transformationMatrix.at<double>(0, 1) = negative_RotationMat_Cv.at<double>(0, 1);
//	trans_transformationMatrix.at<double>(1, 1) = negative_RotationMat_Cv.at<double>(1, 1);
//	trans_transformationMatrix.at<double>(2, 1) = negative_RotationMat_Cv.at<double>(2, 1);
//	trans_transformationMatrix.at<double>(0, 2) = negative_RotationMat_Cv.at<double>(0, 2);
//	trans_transformationMatrix.at<double>(1, 2) = negative_RotationMat_Cv.at<double>(1, 2);
//	trans_transformationMatrix.at<double>(2, 2) = negative_RotationMat_Cv.at<double>(2, 2);
//
//
//	// Umwandlung in euler
//	eulerAngles = rotationMatrixToEulerAngles(negative_RotationMat_Cv);
//
//
//
//
//}
//
//
//void Modell::initialisePointCloudProjection() {
//	
//	// init angles
//	omega = 0.0f;
//	phi = 0.0f;
//	kappa = 0.0f;	
//
//	// calc sensor size in [mm] (using single value for pixel size)
//	x = imageWidth * pixelGr; 
//	y = imageHeight * pixelGr;
//
//	// get principle point and camera constant / focal length
//	x0 = trans_cameraMatrix.at<double>(0, 2);
//	y0 = trans_cameraMatrix.at<double>(1, 2);
//	ck = trans_cameraMatrix.at<double>(0, 0);
//
//	// get distortion parameters
//	A1 = trans_distCoeffs.at<double>(0, 0);
//	A2 = trans_distCoeffs.at<double>(1, 0);
//	A3 = trans_distCoeffs.at<double>(4, 0);
//		
//	B1 = trans_distCoeffs.at<double>(2, 0);
//	B2 = trans_distCoeffs.at<double>(3, 0);
//	
//	// get projection center / translation vector
//	Xo = trans_transformationMatrix.at<double>(0, 3);
//	Yo = trans_transformationMatrix.at<double>(1, 3);
//	Zo = trans_transformationMatrix.at<double>(2, 3);
//
//	// get rotation matrix
//	r11 = trans_transformationMatrix.at<double>(0, 0);
//	r12 = trans_transformationMatrix.at<double>(0, 1);
//	r13 = trans_transformationMatrix.at<double>(0, 2);
//
//	r21 = trans_transformationMatrix.at<double>(1, 0);
//	r22 = trans_transformationMatrix.at<double>(1, 1);
//	r23 = trans_transformationMatrix.at<double>(1, 2);
//
//	r31 = trans_transformationMatrix.at<double>(2, 0);
//	r32 = trans_transformationMatrix.at<double>(2, 1);
//	r33 = trans_transformationMatrix.at<double>(2, 2);
//
//	//recreate projection matrix
//	pixSizeX = x / imageWidth;
//	pixSizeY = y / imageHeight;
//	xMin = x / -2.0f;
//	yMin = y / -2.0f;
//
//	logfile->append(TAG + "rotation matrix: ");
//	logfile->append(TAG + std::to_string(r11) + ", " + std::to_string(r12) + ", " + std::to_string(r13), 4);
//	logfile->append(TAG + std::to_string(r21) + ", " + std::to_string(r22) + ", " + std::to_string(r23), 4);
//	logfile->append(TAG + std::to_string(r31) + ", " + std::to_string(r32) + ", " + std::to_string(r33) + "\n", 4); 
//
//	logfile->append(TAG + "translation vector: ");
//	logfile->append(TAG + std::to_string(Xo) + ", " + std::to_string(Yo) + ", " + std::to_string(Zo) + "\n", 4);
//	
//	logfile->append(TAG + "intrinsics");
//	logfile->append(TAG + "xh: " + std::to_string(x0) + ", yh : " + std::to_string(y0) + ", ck: " + std::to_string(ck) + ", (r0: 0)\n", 4); 
//	
//	logfile->append(TAG + "distortion");
//	logfile->append(TAG + "A1: " + std::to_string(A1) + ", A2 : " + std::to_string(A2) + ", A3: " + std::to_string(A3), 8); // +", A4: " + std::to_string(A4), 8);
//	logfile->append(TAG + "B1: " + std::to_string(B1) + ", B2 : " + std::to_string(B2), 8); // +", B3: " + std::to_string(B3) + ", B4: " + std::to_string(B4), 8);
//	//logfile->append(TAG + "C1: " + std::to_string(C1) + ", C2 : " + std::to_string(C2), 8);
//	logfile->append("\n");
//
//}
//
//void Modell::getColorFor(cv::Point3d& v, cv::Mat& realImage, cv::Vec3b& color) {
//
//
//	cv::Point2d pixel;
//	getImageCoordinates(v, realImage, pixel);
//
//	if (pixel.x > 0 && pixel.y > 0 && pixel.x < realImage.cols && pixel.y < realImage.rows)
//		color = realImage.at<cv::Vec3b>(pixel.y, pixel.x);
//	else
//		color = cv::Vec3b(0, 0, 0); // achtung, setze Farbe zurück auf schwarz!
//
//	//Wasserlinienpunkte reingegeben
//	if (waterlinePoints->size() != 0) {
//
//		cv::Point2d &currentImagePoint = cv::Point2d (pixel.x, pixel.y);
//		if (std::find(waterlinePoints->begin(), waterlinePoints->end(), currentImagePoint) != waterlinePoints->end()) 
//			/* v contains x */
//			waterlinePoints_projected->push_back(cv::Point3d(v.x, v.y, v.z));
//		else {
//			// suche in einer 3x3 Pixelumgebung nach pot. nächstem Nachbarn!
//			for (int c = 0; c < 1; c++) {
//				for (int r = 0; r < 1; r++) {
//					cv::Point2d potentCurrentImagePt = cv::Point2d(pixel.x + c, pixel.y + r);
//					if (std::find(waterlinePoints->begin(), waterlinePoints->end(), potentCurrentImagePt) != waterlinePoints->end()) {
//						/* v contains x */
//						waterlinePoints_projected->push_back(cv::Point3d(v.x, v.y, v.z));
//						return;
//					}
//				}
//			}
//		}
//
//	}
//	
//}
//
//
//
//
//// calculates pixel coordinates for given 3d coordinates 
//void Modell::getImageCoordinates(cv::Point3d& v, cv::Mat &image, cv::Point2d &imagePixel) {
//	
//	
//	
//
//	//Alternative Abbildungsvorschrift für zentralperspektivische Kamera!
//	 // collinearity equations with correction functions //PC ^= PointCloud, X_0 ^= Projektionszentrum
//	double X_PC_minus_X_0 = v.x - Xo;
//	double Y_PC_minus_Y_0 = v.y - Yo;
//	double Z_PC_minus_Z_0 = v.z - Zo;
//
//	
//
//	double X_Kamera = r11 * X_PC_minus_X_0 + r21 * Y_PC_minus_Y_0 + r31 * Z_PC_minus_Z_0;
//	double Y_Kamera = r12 * X_PC_minus_X_0 + r22 * Y_PC_minus_Y_0 + r32 * Z_PC_minus_Z_0;
//	double Z_Kamera = r13 * X_PC_minus_X_0 + r23 * Y_PC_minus_Y_0 + r33 * Z_PC_minus_Z_0;
//
//	
//
//	// Zwishcenkooridnatensystem
//	double x_zwischenKS = X_Kamera / Z_Kamera;
//	double y_zwischenKS = Y_Kamera / Z_Kamera;
//
//
//
//	
//	//erweiteretes Modell der Verzeichnungskorrektur:  implementiert aber erweiterte Parameter auf null erstma setzen! 
//
//	double r_quadrat = (x_zwischenKS * x_zwischenKS) + (y_zwischenKS * y_zwischenKS);
//	double p1 = (A1 * r_quadrat + A2 * r_quadrat * r_quadrat + A3 * r_quadrat * r_quadrat * r_quadrat + A4 * r_quadrat * r_quadrat * r_quadrat * r_quadrat);
//	double delta_x_radial = x_zwischenKS * p1;
//	double delta_y_radial = y_zwischenKS * p1;
//
//	double p2 = 1.0 + B3 * r_quadrat + B4 * r_quadrat * r_quadrat;
//
//	double delta_x_tangential = p2 * (B1 * (r_quadrat + 2.0 * (x_zwischenKS * x_zwischenKS)) + 2.0 * B2 * x_zwischenKS * y_zwischenKS);
//	double delta_y_tangential = p2 * (B2 * (r_quadrat + 2.0 * (x_zwischenKS * x_zwischenKS)) + 2.0 * B1 * x_zwischenKS * y_zwischenKS);
//
//	double delta_x = delta_x_radial + delta_x_tangential;
//	double delta_y = delta_y_radial + delta_y_tangential;
//
//	
//
//	// Anbringen der Korrektur der Objektivverzeichnung: um Verzecihnung korrigierte Zwischenkoordinaten
//	double x_dach = x_zwischenKS + delta_x;
//	double y_dach = y_zwischenKS + delta_y;
//
//	//Bildkooridnaten
//	// Nullpunkt als Mittelpunkt des Sensors betrachten! // x geht nach rechts, y nach oben!
//	// xh, yh ^= BILDhauptpunkt --> Bildkoordinatensystem!
//
//	double xHauptpunkt = x0;
//	double yHauptpunkt = y0;
//
//	
//	// c - Kamerakonstante, C1 - Koefficent für unterschiedliche Pixelausdehnung, C2 Scherungskoefficent
//	double xBild = xHauptpunkt - ck * (1.0 + C1) * x_dach - ck * C2 * y_dach;
//	double yBild = yHauptpunkt - ck * y_dach;
//
//	
//	// Umrechnung in pixelkoordinaten!
//	// Nullpunkt = Pixelmiitelpunkt linken oberen Pixels, x nach rechts, y nach unten
//
//	double breite_px = image.cols; //sensorbreite in pixel!
//	double hoehe_px = image.rows; //sensorhöhe in pixel!
//	
//	double breite_mm = x; // sensorbreite in mm
//	double hoehe_mm = y; // sensorhöhe in mm
//
//
//	//std::cout << "XYZ: " << xHauptpunkt << ", " << yHauptpunkt << ", " << xBild << ", " << yBild << ", " << breite_mm << ", " << hoehe_mm << std::endl;
//
//
//	double x_pixel = xBild * (breite_px / breite_mm) + (breite_px / 2.0) - 0.5;
//	double y_pixel = (hoehe_px / 2.0) - yBild * (hoehe_px / hoehe_mm) - 0.5;
//
//
//	imagePixel.x = (int)x_pixel;
//	imagePixel.y = (int)y_pixel;
//
//	//std::cout << "pixel: " << pixel.x << ", " << pixel.y << std::endl;
//
//	
//		
//	
//}
//
//
//
//// Checks if a matrix is a valid rotation matrix.
//bool Modell::isRotationMatrix(cv::Mat &R)
//{
//	cv::Mat Rt;
//	transpose(R, Rt);
//	cv::Mat shouldBeIdentity = Rt * R;
//	cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());
//
//	return  norm(I, shouldBeIdentity) < 1e-6;
//
//}
//
//// Calculates rotation matrix to euler angles
//// The result is the same as MATLAB except the order
//// of the euler angles ( x and z are swapped ).
//cv::Vec3f Modell::rotationMatrixToEulerAngles(cv::Mat &R)
//{
//
//	assert(isRotationMatrix(R));
//
//	float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
//
//	bool singular = sy < 1e-6; // If
//
//	float x, y, z;
//	if (!singular)
//	{
//		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
//		y = atan2(-R.at<double>(2, 0), sy);
//		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
//	}
//	else
//	{
//		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
//		y = atan2(-R.at<double>(2, 0), sy);
//		z = 0;
//	}
//	return cv::Vec3f(x, y, z);
//
//
//
//}