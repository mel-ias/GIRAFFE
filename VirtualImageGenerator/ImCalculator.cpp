#include "ImCalculator.hpp"
#include<limits>
#include "opencv2\photo.hpp"
#include "DataManager.h"





void ImCalculator::init(DataManager* _dataManager) {

	logfile = _dataManager->getLogFilePrinter(); // init logfile
	logfile->append("");
	logfile->append(TAG + "---- initialisation image calculator ----");

	dataManager = _dataManager;

	_mask = nullptr;
	_image = nullptr;
	
	haveProjectSomething = false;
	colorPoints = false;
	distMax = 0.0f;
	distMin = (std::numeric_limits<float>::max)();
	
	bb = nullptr;

	next_masks.reserve(3);
	next_dists.reserve(3);

	_imageForImFill_knn_Crop = cv::Mat();
	_imageForImFill_inpaint_radius = cv::Mat();

	//_ck = (1.0f);
	//_pixSize = 0.0f;
	//_imageSize = cv::Size(); //initWert!
	//coord_img = nullptr;


	// Settings for camera
	_ck = dataManager->getFocalLength();
	_imageSize = dataManager->get_size_true_image();
	_pixSize = dataManager->get_pixel_size();
	logfile->append(TAG + "received camera parameters (ck, imageSize, pixSize)");
	
	//reference & copy real image
	_realImage = dataManager->get_true_image();
	_realImage.copyTo(_realImage_copy_orig); 
	logfile->append(TAG + "received realImage, create copy (realImage_copy_orig)");


	// create ImCalculator directory
	path_directory_ImCalculator = (dataManager->get_path_working_directory() + "\\ImCalculator\\");
	CreateDirectoryA(LPCSTR(path_directory_ImCalculator.c_str()), NULL);
	logfile->append(TAG + "created ImCalculator directory: " + path_directory_ImCalculator);

	// get tolerance for point projection
	//toleranceForPointProjection = dataManager->getDistanceNoise();


};



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



/*saves all generated synthetic images in ImCalculator directory*/
void ImCalculator::saveImages() {

	logfile->append(TAG + "save images synthetic images...");

	// deallocation of image header and data
	_mask->release();

	// create new mask (imageSize, CV_8UC1)
	_mask = new cv::Mat(_image->rows, _image->cols, CV_8UC1);

	// write dist images and dist pyramids
	cv::imwrite(path_directory_ImCalculator + "dist_image.png", *_distImage);
	if (next_dists.size() > 0) {
		for (unsigned int i = 0; i < next_dists.size(); ++i) {
			std::string name = "dist_image_p";
			name += std::to_string(i) + ".png";

			cv::imwrite(path_directory_ImCalculator + name.c_str(), *next_dists[i]);
		}
	}

	// create CoordinateImage with CimHeader h 
	// use CoordinateImage to store corresponding points (3D point coordinates <-> 2D image coordinates of projected virtual image) 
	CoordinateImage::CimHeader h;

	// width and height will be set by the coordinate image itself
	h.ps = _pixSize;

	// camera projection center in world coordinates corrected by backwardcorrection with x0 = x0 + -k*ke
	h.x0 = bb->get_X0_Cam_World()[0] - bb->get_Correction_backward()* rot_xyz[2];
	h.y0 = bb->get_X0_Cam_World()[1] - bb->get_Correction_backward()* rot_xyz[5];
	h.z0 = bb->get_X0_Cam_World()[2] - bb->get_Correction_backward()* rot_xyz[8];

	// rotation matrix: ie = around x, je = around y, ke = around z
	h.r = RotM(rot_xyz);

	// save _image if it's already a color image
	if (_image->depth() == CV_8U) {

		cv::imwrite(path_directory_ImCalculator + dataManager->get_filename_true_image() + "_synth.png", *_image);
		logfile->append(TAG + "saved mask of virtual image rgb");
		return; // we don't need to calc the gray values as in the next lines
	}

	// clear mask image
	_mask->setTo(cv::Scalar(0));

	// for intensity data 
	for (int i = 0; i < _image->rows; i++) {
		for (int j = 0; j < _image->cols; j++) {
			// get gray value from image
			float intensity = _image->ptr<float>(i)[j]; 
			_mask->at<uchar>(i, j) = (int)floor(intensity * 255 + 0.5);

		}
	}

	//std::cout << TAG << "Mask: " << _mask->cols << "x" << _mask->rows << std::endl;
	//std::cout << TAG << "Image: " << _image->cols << "x" << _image->rows << std::endl;

	// save image as *.im.png
	cv::imwrite(path_directory_ImCalculator + dataManager->get_filename_true_image() + ".png", *_mask);
	logfile->append(TAG + "saved mask of virtual image intensity");


}



/*Projekt this point to the current images*/
void ImCalculator::projectPoint(LaserPoint* lp) {

	
	
	float dx = lp->_xyz[0]- bb->get_X0_Cam_World()[0]; //^= X_PC_minus_X0 -> Frank
	float dy = lp->_xyz[1]- bb->get_X0_Cam_World()[1]; //^= Y_PC_minus_Y0 -> Frank
	float dz = lp->_xyz[2]- bb->get_X0_Cam_World()[2]; //^= Z_PC_minus_Z0 -> Frank
	float x = rot_xyz[0] * dx + rot_xyz[3] * dy + rot_xyz[6] * dz; // ^= x_Kamera -> Frank
	float y = rot_xyz[1] * dx + rot_xyz[4] * dy + rot_xyz[7] * dz; // ^= Y_Kamera -> Frank
	float z = rot_xyz[2] * dx + rot_xyz[5] * dy + rot_xyz[8] * dz; // ^= Z_Kamera -> Frank

																  
	// Prüfe ob der zu projizierende Punkt zu nah am Projektionszentrum liegt, wenn loc_accuracy vergeben wurde. 1m standardmäßig abstand halten!
	//float eucl_Distance = sqrt(sq(dx) + sq(dy) + sq(dz));
	//if (eucl_Distance < toleranceForPointProjection)
	//	return;
	//std::cout << TAG << "Tolerance for eucl. distance object<->projC = 3.0 m" << std::endl;
		

	// check near and far
	if (z < /*0.0f*/ dataManager->getThresholdForPointProjection() || z > bb->get_dist()) return;
	
	//z += bb->get_Correction_backward();

	
	// _ck in mm, Hauptpunkt als Abweichung von ittelpunkt!
	float u = _ck  * x / z;
	float v = _ck * y / z;

	float column_float = (u - image_plane[0]) / _pixSize;
	float row_float = (v - image_plane[2]) / _pixSize;

	
	// calc to column and row
	int column = static_cast<int>(floor(column_float)); // column
	int row = static_cast<int>(floor(row_float)); // row

	//// END OLD ////


	// check the Image borders
	if (row < 0 || row >(rows - 1) || column < 0 || column >(columns - 1)) return;

	haveProjectSomething = true;

	float dist = static_cast<float>(sqrt(dx*dx + dy*dy + dz*dz));

	// check for extrema dist

	if (dist < distMin) distMin = dist;
	if (dist > distMax) distMax = dist;

	//unsigned char* pixel = nullptr;
	cv::Point3_ <uchar>* pixel;
	// if there is already a value at mask(row, column)
	if (_mask->ptr<float>(row)[column] > 0.0f) {
		//if (((float*)(_mask->imageData + row*_mask->widthStep))[column] > 0.0f) {

		// if the new distance is smaller than the value in mask(row,column)
		if (dist < _mask->ptr<float>(row)[column]) {
			//if (dist < ((float*)(_mask->imageData + row*_mask->widthStep))[column]) {

			// replace distance value with this 
			_mask->ptr<float>(row)[column] = dist;
			//((float*)(_mask->imageData + row*_mask->widthStep))[column] = dist;

			// replace image value with this one

			if (colorPoints) {
				pixel = _image->ptr<cv::Point3_<uchar>>(row, column);
				pixel->x = lp->color[2];
				pixel->y = lp->color[1];
				pixel->z = lp->color[0];
				
				dataManager->get_coordinate_image()->setPixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float));

			}
			else {
				_image->ptr<float>(row)[column] = lp->_intensity;

				dataManager->get_coordinate_image()->setPixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], lp->_intensity, column_float, row_float));
			}
		
		}
	}
	else {
		// otherwise there is nothing yet, so just assign the  values
	
		_mask->ptr<float>(row)[column] = dist;

		if (colorPoints) {
			pixel = _image->ptr<cv::Point3_<uchar>>(row, column);
			pixel->x = lp->color[2];
			pixel->y = lp->color[1];
			pixel->z = lp->color[0];
		

			dataManager->get_coordinate_image()->setPixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float));

		}
		else {
			_image->ptr<float>(row)[column] = lp->_intensity;
			dataManager->get_coordinate_image()->setPixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], lp->_intensity, column_float, row_float));

		}
	
	}


	// calc _mask for the next pyramids
	for (unsigned int p = 0u; p < next_masks.size(); ++p) {
		float scale = powf(2.0f, 1.0f*(p + 1u));


		int cPyr = static_cast<int>(floor(((u - image_plane[0]) / (scale*_pixSize))));
		int rPyr = static_cast<int>(floor(((v - image_plane[2]) / (scale*_pixSize))));



		float mask_pix = next_masks[p]->ptr<float>(rPyr)[cPyr];
		if (mask_pix > 0.0f) {
			if (mask_pix > dist) {
				next_masks[p]->ptr<float>(rPyr)[cPyr] = dist;
				
			}
		}
		else {
			next_masks[p]->ptr<float>(rPyr)[cPyr] = dist;
			
		}
	}

	
}


/*Init Images with the given bb*/
/*float xbmin, float xbmax, float zbmin, float zbmax, float yb,float* Rz*/
void ImCalculator::init_Image(BoundingBox* b) {

	bb = b;

	calc_image_Plane(image_plane);

	columns = static_cast<int>(ceil((image_plane[1] - image_plane[0]) / _pixSize)) + 1;
	rows = static_cast<int>(ceil((image_plane[3] - image_plane[2]) / _pixSize)) + 1;

	init_images(columns, rows);
	logfile->append(TAG + "initalize image_plane (px): " + std::to_string(columns) + "x" + std::to_string(rows));

}


/*This calculate the image Plane as an projektion from the bounding box far area.
It also sets ie,je and ke*/
void ImCalculator::calc_image_Plane(float* plane /*,float xbmin, float xbmax, float zbmin, float zbmax, float yb,float *Rz*/) {


	// we have the BB in kamera koordinates, so the calculation from the image plane is simple:
	// koordinates in the camerasystem
	float xk, yk, zk;

	// the upper left edge (u0,v0)
	xk = bb->get_xMin();
	yk = -bb->get_zMax();
	zk = bb->get_yMax();// +bb->get_Correction_backward();


	plane[0] = xk / zk *_ck;
	plane[2] = yk / zk *_ck;

	// the lower right edge (u1,v1)
	xk = bb->get_xMax();
	yk = -bb->get_zMin();

	plane[1] = xk / zk *_ck;
	plane[3] = yk / zk *_ck;

	// take rotation matrix from bounding box. in case of android rot_m, 
	// rotation matrix is already provided. otherwise rotation matrix would be calculated during json imread
	rot_xyz = dataManager->getRotationMatrix();

	logfile->append("");
	logfile->append(TAG + "TVec:");
	logfile->append("\t\t" + std::to_string(bb->get_X0_Cam_World()[0]) + " " + std::to_string(bb->get_X0_Cam_World()[1]) + " " + std::to_string(bb->get_X0_Cam_World()[2]));

	logfile->append(TAG + "RotM:");
	logfile->append("\t\t" + std::to_string(rot_xyz[0]) + " " + std::to_string(rot_xyz[3]) + " " + std::to_string(rot_xyz[6]),4);
	logfile->append("\t\t" + std::to_string(rot_xyz[1]) + " " + std::to_string(rot_xyz[4]) + " " + std::to_string(rot_xyz[7]),4);
	logfile->append("\t\t" + std::to_string(rot_xyz[2]) + " " + std::to_string(rot_xyz[5]) + " " + std::to_string(rot_xyz[8]),4);
	logfile->append("");
	logfile->append(TAG + "calculated plane: " + std::to_string(plane[0]) + "," + std::to_string(plane[1]) + "," + std::to_string(plane[2]) + "," + std::to_string(plane[3]), 4);
}


/*Initialize all nessecary images with the given coloumns and rows.
The size of the pyramids will also be calculate.*/
void ImCalculator::init_images(int column, int row) {


	if (colorPoints) {
		_image = new cv::Mat(row, column, CV_8UC3);
		_image->setTo(cv::Scalar(255, 255, 255));
		
	}
	else {
	
		_image = new cv::Mat(row, column, CV_32FC1);
		_image->setTo(cv::Scalar(255, 255, 255));
	}

	_mask = new cv::Mat(row, column, CV_32FC1);
	_mask->setTo(cv::Scalar(-1.0f));
	
	_distImage = new cv::Mat(row, column, CV_8UC3);
	_distImage->setTo(cv::Scalar(0u, 0u, 0u));

	// generate next 3 pyramids 
	for (int i = 1; i <= 3; ++i) {
		int scale = static_cast<int>(powf(2.0f, i*1.0f));
		int pc = column % scale == 0 ? (column / scale) : (column / scale + 1); // columns in pyramid ( calcs the next integral f.e. 2.5 -> 3)
		int pr = row % scale == 0 ? (row / scale) : (row / scale + 1); // rows in pyramid

		next_dists.push_back(new cv::Mat(pr, pc, CV_8UC3)); 
		next_dists[i - 1u]->setTo(cv::Scalar(0u, 0u, 0u));

		next_masks.push_back(new cv::Mat(pr, pc, CV_32FC1));
		next_masks[i - 1u]->setTo(cv::Scalar(-1.0f));

	}

	dataManager->set_coordinate_image(column, row);

}

/*This do all the stuff, needed to generate the output images.
CAUTION call this after the projection of all points!!*/
void ImCalculator::writeImages() {

	if (!haveProjectSomething) return;

	distMax -= distMin;

	calc_distImage(distMin, distMax, false);


	cv::imwrite(path_directory_ImCalculator + "dist_image_orig.png", *_distImage);
	logfile->append(TAG + "calc new masks");


	//filter image, new calc distance image and pyramids
	filter_image(1.0f);
	calc_distImage(distMin, distMax);
	calc_distPyramids(distMin, distMax);

}

/*This calcs the colors of the distImage with the current data in _mask.
deleteCurrentData say if information in the distImage, which is deleted in_mask will be reset to 0 or not.*/
void ImCalculator::calc_distImage(float d_min, float d_diff, bool deleteCurrentData) {

	logfile->append(TAG + "save distances.. ");
	for (int r = 0; r < _distImage->rows; ++r) {
		for (int c = 0; c < _distImage->cols; ++c) {

			cv::Point3_ <uchar>* pixel = _distImage->ptr<cv::Point3_<uchar>>(r, c);

			// get distance
			//float d = reinterpret_cast<float*>(_mask->imageData + r*_mask->widthStep)[c];
			float d = _mask->ptr<float>(r)[c];


			// calc distance between [0..1]
			// ignore default values
			if (d > 0.0f) {
				d = (d - d_min) / d_diff;
				// calc blend values for red, green and blue
				float red = d <= 0.5f ? (1.0f - 2.0f * d) : 0.0f; // red = [1..0] , if d=[0..0.5] else 0.0
				float green = d <= 0.5f ? 2.0f*d : (2.0f - 2.0f*d); // green = [0..1], if d=[0..0.5] else [1..0]
				float blue = d >= 0.5f ? (2.0f*d - 1.0f) : 0.0f; // blue = 0.0 if d=[0..0.5] else [0..1]


				pixel->x = static_cast<unsigned char>(blue * 255u);
				pixel->y = static_cast<unsigned char>(green * 255u);
				pixel->z = static_cast<unsigned char>(red * 255u);
			}
			else if (deleteCurrentData) {

				pixel->x = 0u;
				pixel->y = 0u;
				pixel->z = 0u;

			}
		}
	}

	logfile->append(TAG + "done.");

}

/*Filter Methode, um überlagerte Hintergrund Pixel aus dem Vordergrund zu eliminieren.
Hierfür werden die Pyramiden Bilder genutzt und neu die Hintergrund pixel schrittwiese gelöscht.
db gibt den maximalen Abstand zwischen einem Vordergrund und einem Hintergrund Pixel an.
Wenn dieser Abstand überschritten wird, wird das Hintergrund pixel gelöscht.*/
void ImCalculator::filter_image(float db) {

	// pyr mask, that will be filtered.
	cv::Mat* nextPyr = 0;

	for (int pyr_index = next_masks.size() - 1; pyr_index >= 0; --pyr_index) {

		cv::Mat* current_pyr = next_masks[pyr_index];

		if (pyr_index != 0)
			nextPyr = next_masks[pyr_index - 1];
		else
			nextPyr = _mask;

		for (int r = 0; r < nextPyr->rows; ++r) {
			for (int c = 0; c < nextPyr->cols; ++c) {

				float d = nextPyr->ptr<float>(r)[c];

				int cPyr = c / 2;
				int rPyr = r / 2;

				// get d from current Pyr_mask
				float dPyr = current_pyr->ptr<float>(rPyr)[cPyr];


				if (d == -1.0f)
					continue;

				if (abs(d - dPyr) > db) {

					/*
					Dieser Punkt soll evtl. gelöscht werden.
					Nun wird geprüft, ob es sich um EInen Punkt an einer Kante handelt.
					Kanten Punkte sind wie folgt definiert:
					r/c = ungerade : in der aktuellen Pyr muss der nächste Pixel passen
					r/c = gerade: in der aktuellen Pyr muss der vorherige Pixel passen.
					*/

					bool checkR = true;
					bool checkC = true;
					float dNew = 0.0f;

					// check r. anfangs und Endzeile wird übersprungen
					if (r < nextPyr->rows - 1 && r > 0) {
						if (r % 2 == 0) {

							dNew = current_pyr->ptr<float>(rPyr - 1)[cPyr - 1];

							if (dNew > 0.0f)
								checkR = abs(d - dNew) > db;
						}
						else {

							dNew = current_pyr->ptr<float>(rPyr + 1)[cPyr + 1];
							if (dNew > 0.0f)
								checkR = abs(d - dNew) > db;
						}
					}

					// check c.
					if (c < nextPyr->cols - 1 && c > 0) {
						if (c % 2 == 0) {

							dNew = current_pyr->ptr<float>(rPyr)[cPyr - 1];
							if (dNew > 0.0f)
								checkC = abs(d - dNew) > db;
						}
						else {

							dNew = current_pyr->ptr<float>(rPyr)[cPyr + 1];
							if (dNew > 0.0f)
								checkC = abs(d - dNew) > db;
						}
					}

					// Lösche, wenn kein Randpixel getroffen wurde
					if (checkR && checkC) {
						nextPyr->ptr<float>(r)[c] = -1.0f;
						if (pyr_index == 0) {
							if (r > 0 && r < _image->rows && c > 0 && c < _image->cols) {
								if (colorPoints) {
									cv::Point3_ <uchar>* pixel = _image->ptr<cv::Point3_<uchar>>(r, c);
									pixel->x = 255u;
									pixel->y = 255u;
									pixel->z = 255u;
								}
								else {
									_image->ptr<float>(r)[c] = 1.0f;
								}
								dataManager->get_coordinate_image()->deletePixel(c, r);
							}
						}
					}
				}
			}
		}
	}
}

/*Calc the color for all distance pyramids*/
void ImCalculator::calc_distPyramids(float d_min, float d_diff) {

	for (unsigned int p = 1u; p <= next_dists.size(); ++p) {
		for (int r = 0; r < next_dists[p - 1u]->rows; ++r) {
			for (int c = 0; c < next_dists[p - 1u]->cols; ++c) {

				cv::Point3_ <uchar>* pixel = next_dists[p - 1u]->ptr<cv::Point3_<uchar>>(r, c);

				// get distance
				float d = next_masks[p - 1u]->ptr<float>(r)[c];
				// calc distance between [0..1]
				// ignore default values
				if (d > 0) {
					d = (d - d_min) / d_diff;
					// calc blend values for red, green and blue
					float red = d <= 0.5f ? (1.0f - 2.0f * d) : 0.0f; // red = [1..0] , if d=[0..0.5] else 0.0
					float green = d <= 0.5f ? 2.0f*d : (2.0f - 2.0f*d); // green = [0..1], if d=[0..0.5] else [1..0]
					float blue = d >= 0.5f ? (2.0f*d - 1.0f) : 0.0f; // blue = 0.0 if d=[0..0.5] else [0..1]

					pixel->x = static_cast<unsigned char>(blue * 255u);
					pixel->y = static_cast<unsigned char>(green * 255u);
					pixel->z = static_cast<unsigned char>(red * 255u);
				}
			}
		}
	}
}



void ImCalculator::fillVektorsForImFill() {

	//double shifter_x = 0.0, shifter_y = 0.0; // Reicht lokal aus!

	std::vector<Vek2d>* punkte = dataManager->get_pts_synth_2D_double();
	std::vector<Vek3i>* farbenPunkte = dataManager->get_pts_color_RGB_int();
	std::vector<Vek3d>* punkte3D = dataManager->get_pts_synth_3D_double();

	// prüfe ob vektoren angelegt worden sind, sonst springe aus funktion
	if (punkte == nullptr || farbenPunkte == nullptr || punkte3D == nullptr) {
		logfile->append(TAG + "Vectors for points, 3d points and color saving not inialised! return.");
		return;
	}

	//prüfe ob Vektoren bei Input gefüllt sind und lösche sie ggf.
	if (punkte->size() != 0) {
		punkte->clear(); farbenPunkte->clear(); punkte3D->clear();
	}

	//hole pixel raus
	//std::vector<CoordinateImage::Coordinate*> pixels = coord_img->getPixels();
	std::vector<CoordinateImage::Coordinate*> pixels = dataManager->get_coordinate_image()->getPixels();
	auto end = pixels.cend();


	uint counter = 0;

	for (auto it = pixels.cbegin(); it != end; ++it) {
		if (*it == nullptr) {
			 // 0 char to show, that here is no data
			
		}
		else {


			punkte->push_back(Vek2d((*it)->xi, (*it)->yi));
			punkte3D->push_back(Vek3d((*it)->x, (*it)->y, (*it)->z)); 


			if (colorPoints)
				farbenPunkte->push_back(
					Vek3i(
						static_cast<unsigned int>((*it)->color[0]),
						static_cast<unsigned int>((*it)->color[1]),
						static_cast<unsigned int>((*it)->color[2])));
			else {

				//schiebe signed int rein um float zu sparen!
				farbenPunkte->push_back(Vek3i(static_cast<int> ((*it)->intensity * 100), static_cast<int> ((*it)->intensity * 100), static_cast<int> ((*it)->intensity * 100)));
			}

			counter++;

		}
	}
}



void ImCalculator::fillImage(size_t k_for_knn) {

	logfile->append(TAG + "Fill Images ... ");

	// Init
	_maske_8UC1 = cv::Mat::zeros(_image->size(), CV_8UC1);


	//Prüfe auf Anzahl der Kanäle
	if (_image->channels() == 1) {

		// 1. rechne Wertebereich um Rechenzeit zu sparen --> nehme letzten 2 Nachkommastellen mit und rechne Matrix *100
		// caste nun float Bild auf 16 Bit signed int

		cv::Mat _newImageMat = cv::Mat::zeros(_image->size(), _image->type());
		cv::Mat _maske_32SC1 = cv::Mat::zeros(_image->size(), CV_32SC1);


		for (int r = 0; r < _image->rows; r++)
			for (int c = 0; c < _image->cols; c++) {

				if (_image->ptr<float>(r)[c] != 1) {
					_newImageMat.ptr<float>(r)[c] = _image->ptr<float>(r)[c] * 100;
					_maske_8UC1.ptr<uchar>(r)[c] = 1;
					_maske_32SC1.ptr<unsigned int>(r)[c] = 1;
				}
			}

		_newImageMat.convertTo(*_image, CV_32SC1);
		_image->mul(_maske_32SC1);

		// Speichere und überschreibe adjMap und falseColorsMap
		colormappingIntensity(*_image, _maske_8UC1, cv::Rect(), "ImageUnfilled", adjMap, falseColorsMap);



		_imageForImFill_inpaint_knn = *_image;

		logfile->append(TAG + "Typ: " + std::to_string(_image->type()));

		for (int r = 0; r < _image->rows; r++)
			for (int c = 0; c < _image->cols; c++) {
				if (_image->ptr<ushort>(r, c)[0] != 0)
					cv::circle(_maske_8UC1, cv::Point2d(c, r), 2, cv::Scalar(255), -1);
			}
		cv::imwrite(path_directory_ImCalculator + "maskefill.png", _maske_8UC1);

		// calc visualisation
		berechneVisualisierung32SC1(_imageForImFill_inpaint_knn, _maske_8UC1, (*dataManager->get_pts_synth_2D_double()), (*dataManager->get_pts_color_RGB_int()), k_for_knn);

		// Konvertierung der Intensitäten 
		double _min, _max;
		cv::minMaxIdx(_imageForImFill_inpaint_knn, &_min, &_max);
	

		// range ^= kompletter Anzahl an Intensitäten --> abs(min) + abs(max)
		int range = abs(_min) + abs(_max);
		int range005 = range * 0.50;

		//rechne von minimalstem Wert + range 0.05 um ausgrenzbereich zu definieren und von maximalem Bereich - 0.05 prozent
		_max -= range005;

		int counter = 0;
		// prüfe ob intensitäten in diesem intervall liegen, ansonsten lösche diese raus/ setze auf null:
		for (int r = 0; r < _imageForImFill_inpaint_knn.rows; r++)
			for (int c = 0; c < _imageForImFill_inpaint_knn.cols; c++)
				if (_imageForImFill_inpaint_knn.ptr<int>(r)[c] < _min || _imageForImFill_inpaint_knn.ptr<int>(r)[c] > _max) {
					_imageForImFill_inpaint_knn.ptr<int>(r)[c] = 0;
					_maske_8UC1.ptr<uchar>(r)[c] = 0;
					_maske_32SC1.ptr<unsigned int>(r)[c] = 0;
					counter++;
				}

		
		// Wertebereichtransformation auf 0 bis max+min
		if (_min < 0)
			_imageForImFill_inpaint_knn += ((-1) * _min);
		else
			_imageForImFill_inpaint_knn -= _min;

		_imageForImFill_inpaint_knn.copyTo(_imageForImFill_inpaint_knn, _maske_8UC1);
		//_imageForImFill_radius.copyTo(_imageForImFill_radius_Crop, _maske);


		int largest_area = 0;
		int largest_contour_index = 0;
		cv::Rect bounding_rect;
		std::vector<std::vector<cv::Point> > contours; // Vector for storing contours

		findContours(_maske_8UC1, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE); // Find the contours in the image

		for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
		{
			double area = contourArea(contours[i]);  //  Find the area of contour
			if (area > largest_area) {
				largest_area = area;
				largest_contour_index = i;               //Store the index of largest contour
				bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
			}
		}

		// Speichere und überschreibe adjMap und falseColorsMap
		colormappingIntensity(_imageForImFill_inpaint_knn, _maske_8UC1, bounding_rect, "imageForImFill_Inpaint", adjMap, falseColorsMap);


		// Überschreibe datei für sift!
		adjMap.copyTo(_imageForImFill_inpaint_knn);



	}
	else {

		cv::Mat _imageForImFill_knn = cv::Mat(_image->size(), CV_8UC3);
		//cv::Mat _imageForImFill_radius = cv::Mat(_imageMat.size(), CV_8UC3);
		_imageForImFill_knn = *_image;
		//_imageMat.copyTo(_imageForImFill_radius);


		for (int r = 0; r < _image->rows; r++)
			for (int c = 0; c < _image->cols; c++) {
				if (
					!(
						_image->ptr<cv::Vec3b>(r)[c][0] == 255 &&
						_image->ptr<cv::Vec3b>(r)[c][1] == 255 &&
						_image->ptr<cv::Vec3b>(r)[c][2] == 255)
					)
					cv::circle(_maske_8UC1, cv::Point2d(c, r), 10, cv::Scalar(255), -1);
			}

		//berechne gefülltes Bild mit knn und radius basierter Version
		cv::imwrite(path_directory_ImCalculator + "maskefill.png", _maske_8UC1);
		//float radius = 3;

		berechneVisualisierung(_imageForImFill_knn, _maske_8UC1, (*dataManager->get_pts_synth_2D_double()), (*dataManager->get_pts_color_RGB_int()), k_for_knn);
		//berechneVisualisierung(_imageForImFill_radius, _maske, punkte, farbenPunkte, radius);

		//Crop image -> nutze Maske dafür und suche nach größtem zusammengehörigen Bereich --> verwerfe kleinscheiß..
		_imageForImFill_knn.copyTo(_imageForImFill_knn_Crop, _maske_8UC1);
		//_imageForImFill_radius.copyTo(_imageForImFill_radius_Crop, _maske);

		int largest_area = 0;
		int largest_contour_index = 0;
		cv::Rect bounding_rect;
		std::vector<std::vector<cv::Point> > contours; // Vector for storing contours

		findContours(_maske_8UC1, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE); // Find the contours in the image

		for (size_t i = 0; i < contours.size(); i++) // iterate through each contour.
		{
			double area = contourArea(contours[i]);  //  Find the area of contour
			if (area > largest_area) {
				largest_area = area;
				largest_contour_index = i;               //Store the index of largest contour
				bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
			}
		}
		//_imageForImFill_knn_Crop = _imageForImFill_knn_Crop(bounding_rect);

		cv::imwrite(path_directory_ImCalculator + "filledImage_knn.png", _imageForImFill_knn_Crop);
		//cv::imwrite("filledImage_radius.png", _imageForImFill_radius_Crop);

		// Übetrage für leichten zugriff imfillknn in imfillinpaint
		_imageForImFill_inpaint_knn = _imageForImFill_knn_Crop;


	}

	logfile->append(TAG + "write filled and cropped image");


}

// Croppt Bild mit Rechteck wenn vorhanden
void ImCalculator::colormappingIntensity(const cv::Mat& mat, const cv::Mat& maske, const cv::Rect& rectangleToCrop, const std::string& fileName, cv::Mat& adjMap, cv::Mat &falseColorsMap) {

	double min, max, scale;
	cv::minMaxIdx(mat, &min, &max);
	logfile->append(TAG + "color mapping intensity: " + fileName + ", min: " + std::to_string(min) + ", max: " + std::to_string(max));

	scale = 255 / (max - min); // Histogram Equalization

	mat.convertTo(adjMap, CV_8UC1, scale, -min*scale);
	cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

	cv::Mat copyMaske;
	cv::cvtColor(maske, copyMaske, cv::COLOR_GRAY2RGB);
	falseColorsMap = falseColorsMap.mul(copyMaske);

	// wenn Rechteck übergeben, croppe es
	if (!rectangleToCrop.empty()) {
		cv::Mat adjMap2 = adjMap(rectangleToCrop);
		cv::Mat falseColor2 = falseColorsMap(rectangleToCrop);

		adjMap2.copyTo(adjMap);
		falseColor2.copyTo(falseColorsMap);
	}

	// wenn Name übergeben wurde, schreibe Bild sonst nicht 
	if (!fileName.empty()) {
		cv::imwrite(path_directory_ImCalculator + fileName + "_falseColor.png", falseColorsMap);
		cv::imwrite(path_directory_ImCalculator + fileName + "_grayscale.png", adjMap);
	}	
}