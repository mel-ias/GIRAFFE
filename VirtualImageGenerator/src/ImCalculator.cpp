#include "ImCalculator.hpp"
#include <limits>
#include "opencv2\photo.hpp"
#include "DataManager.h"



void ImCalculator::init(DataManager* _dataManager) {

	logfile = _dataManager->getLogFilePrinter(); // init logfile
	logfile->append("");
	logfile->append(TAG + "---- initialisation image calculator ----");

	dataManager = _dataManager;

	_mask = nullptr;
	_image = nullptr;
	
	distMax = 0.0f;
	distMin = (std::numeric_limits<float>::max)();
	
	bb = nullptr;

	next_masks.reserve(3);
	next_dists.reserve(3);

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
	h.r = rot_xyz;

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

	// save image as *.im.png
	cv::imwrite(path_directory_ImCalculator + dataManager->get_filename_true_image() + ".png", *_mask);
	logfile->append(TAG + "saved mask of virtual image intensity");


}










/*Projekt this point to the current images*/
void ImCalculator::projectPoint(LaserPoint* lp) {

	
	
	float dx = lp->_xyz[0]- bb->get_X0_Cam_World()[0]; 
	float dy = lp->_xyz[1]- bb->get_X0_Cam_World()[1]; 
	float dz = lp->_xyz[2]- bb->get_X0_Cam_World()[2]; 
	float x = rot_xyz[0] * dx + rot_xyz[3] * dy + rot_xyz[6] * dz; 
	float y = rot_xyz[1] * dx + rot_xyz[4] * dy + rot_xyz[7] * dz; 
	float z = rot_xyz[2] * dx + rot_xyz[5] * dy + rot_xyz[8] * dz; 

	
	// check near and far
	if (z < dataManager->getThresholdForPointProjection() || z > bb->get_dist()) return;
	
	// _ck in mm, Hauptpunkt als Abweichung von ittelpunkt!
	float u = _ck  * x / z;
	float v = _ck * y / z;

	float column_float = (u - image_plane[0]) / _pixSize;
	float row_float = (v - image_plane[2]) / _pixSize;
	
	// calc to column and row
	int column = static_cast<int>(floor(column_float)); // column
	int row = static_cast<int>(floor(row_float)); // row

	// check the Image borders
	if (row < 0 || row >(rows - 1) || column < 0 || column >(columns - 1)) return;

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


			pixel = _image->ptr<cv::Point3_<uchar>>(row, column);
			pixel->x = lp->color[2];
			pixel->y = lp->color[1];
			pixel->z = lp->color[0];
				
			dataManager->get_coordinate_image()->set_pixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float));

			
		
		}
	}
	else {
		// otherwise there is nothing yet, so just assign the  values
	
		_mask->ptr<float>(row)[column] = dist;


		pixel = _image->ptr<cv::Point3_<uchar>>(row, column);
		pixel->x = lp->color[2];
		pixel->y = lp->color[1];
		pixel->z = lp->color[0];
		

		dataManager->get_coordinate_image()->set_pixel(column, row, CoordinateImage::Coordinate(lp->_xyz[0], lp->_xyz[1], lp->_xyz[2], pixel, column_float, row_float));

		
	
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



	_image = new cv::Mat(row, column, CV_8UC3);
	_image->setTo(cv::Scalar(255, 255, 255));
		
	

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
								
								cv::Point3_ <uchar>* pixel = _image->ptr<cv::Point3_<uchar>>(r, c);
								pixel->x = 255u;
								pixel->y = 255u;
								pixel->z = 255u;
								
								dataManager->get_coordinate_image()->delete_pixel(c, r);
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



void ImCalculator::fill_vectors() {
	// Retrieve vectors from the data manager
	std::vector<Vek2d>* synth2DCoordinates = dataManager->get_pts_synth_2D_double();
	std::vector<Vek3i>* synth3DColors = dataManager->get_pts_color_RGB_int();
	std::vector<Vek3d>* synth3DCoordinates = dataManager->get_pts_synth_3D_double();

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
	std::vector<std::shared_ptr<CoordinateImage::Coordinate>> pixelCoordinates = dataManager->get_coordinate_image()->getPixels();
	auto endIt = pixelCoordinates.cend();


	// Iterate through each pixel coordinate
	for (auto it = pixelCoordinates.cbegin(); it != endIt; ++it) {
		// Check if the current pixel coordinate is not null
		if (*it) { // Using the shared_ptr's conversion to bool
			// Add 2D coordinates to the vector
			synth2DCoordinates->emplace_back(Vek2d((*it)->xi, (*it)->yi));
			// Add 3D coordinates to the vector
			synth3DCoordinates->emplace_back(Vek3d((*it)->x, (*it)->y, (*it)->z));
			// Add RGB color to the vector
			synth3DColors->emplace_back(
				Vek3i(
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

	
	// Initialize the mask with zeros (black)
	_maske_8UC1 = cv::Mat::zeros(_image->size(), CV_8UC1);

	// Loop through each pixel to populate the mask
	for (int r = 0; r < _image->rows; r++) {
		for (int c = 0; c < _image->cols; c++) {
			// Check if the pixel is not white
			if (_image->at<cv::Vec3b>(r, c) != cv::Vec3b(255, 255, 255)) {
				cv::circle(_maske_8UC1, cv::Point(c, r), radius_mask_fill, cv::Scalar(255), -1);
			}
		}
	}

	// Save the mask to file for verification
	cv::imwrite(path_directory_ImCalculator + "maskefill.png", _maske_8UC1);


	assert(!punkte.empty());
	assert(farbenPunkte.size() == punkte.size());
	assert(!bild.empty());
	assert(bild.rows > 0 && bild.cols > 0);
	assert(bild.rows == maske.rows && bild.cols == maske.cols);
	assert(maske.channels() == 1);

	// Schritt 1: Die Punktefarben direkt ins Bild einfügen (parallelisiert)
	cv::Mat tempBild = (*_image).clone();
	cv::Mat fillMask = cv::Mat::zeros(_maske_8UC1.size(), CV_8U);  // Neue Maske für Interpolation

#pragma omp parallel for
	for (int i = 0; i < (*dataManager->get_pts_synth_2D_double()).size(); ++i) {
		int x = static_cast<int>((*dataManager->get_pts_synth_2D_double())[i].x());
		int y = static_cast<int>((*dataManager->get_pts_synth_2D_double())[i].y());

		if (x >= 0 && x < tempBild.cols && y >= 0 && y < tempBild.rows) {
			cv::Vec3b& pixel = tempBild.at<cv::Vec3b>(y, x);
			pixel[0] = static_cast<uchar>((*dataManager->get_pts_color_RGB_int())[i][0]);
			pixel[1] = static_cast<uchar>((*dataManager->get_pts_color_RGB_int())[i][1]);
			pixel[2] = static_cast<uchar>((*dataManager->get_pts_color_RGB_int())[i][2]);

#pragma omp critical  // Da mehrere Threads hier auf die Maske zugreifen
			{
				fillMask.at<uchar>(y, x) = 255;  // Diese Punkte als gefüllt markieren
			}
		}
	}

	// Schritt 2: Lücken füllen mit bilinearer Interpolation oder Region Growing (parallelisiert)
#pragma omp parallel for collapse(2)
	for (int row = 0; row < (*_image).rows; ++row) {
		for (int col = 0; col < (*_image).cols; ++col) {
			if (_maske_8UC1.at<uchar>(row, col) > 0 && fillMask.at<uchar>(row, col) == 0) {
				// Hole benachbarte Pixel und führe eine bilineare Interpolation durch
				cv::Vec3f interpolierteFarbe = cv::Vec3f(0, 0, 0);
				int nCount = 0;

				// Nachbarn betrachten (Region Growing Ansatz)
				for (int yOff = -1; yOff <= 1; ++yOff) {
					for (int xOff = -1; xOff <= 1; ++xOff) {
						int ny = row + yOff;
						int nx = col + xOff;

						if (ny >= 0 && ny < (*_image).rows && nx >= 0 && nx < (*_image).cols &&
							fillMask.at<uchar>(ny, nx) == 255) {
							// Gewichtete Interpolation basierend auf der Entfernung
							cv::Vec3b nachbarFarbe = tempBild.at<cv::Vec3b>(ny, nx);
							interpolierteFarbe += cv::Vec3f(nachbarFarbe[0], nachbarFarbe[1], nachbarFarbe[2]);
							nCount++;
						}
					}
				}

				if (nCount > 0) {
					interpolierteFarbe /= nCount;  // Durchschnitt der Farben der Nachbarn

					// Setze die Farbe im Bild
					tempBild.at<cv::Vec3b>(row, col) = cv::Vec3b(
						cv::saturate_cast<uchar>(interpolierteFarbe[0]),
						cv::saturate_cast<uchar>(interpolierteFarbe[1]),
						cv::saturate_cast<uchar>(interpolierteFarbe[2])
					);

#pragma omp critical  // Da mehrere Threads auf fillMask zugreifen
					{
						// Markiere diesen Pixel als gefüllt
						fillMask.at<uchar>(row, col) = 255;
					}
				}
			}
		}
	}

	// Schritt 3: Setze nicht gefüllte Bereiche auf Schwarz
#pragma omp parallel for collapse(2)
	for (int row = 0; row < (*_image).rows; ++row) {
		for (int col = 0; col < (*_image).cols; ++col) {
			if (_maske_8UC1.at<uchar>(row, col) > 0 && fillMask.at<uchar>(row, col) == 0) {
				// Setze den Pixel auf Schwarz
				tempBild.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
			}
		}
	}

	// Schritt 4: Apply Erosion to filled image
	cv::Mat erodeElement = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius_mask_fill, radius_mask_fill));
	cv::erode(_maske_8UC1, _maske_8UC1, erodeElement);
	tempBild.setTo(cv::Scalar(0, 0, 0), _maske_8UC1 == 0);

	// Schritt 5: Gefülltes Bild zurück ins ursprüngliche Bild kopieren
	tempBild.copyTo(*_image);

	// Save the filled image to file
	cv::imwrite(path_directory_ImCalculator + "filled_image.png", *_image);
	logfile->append(TAG + "write filled_image.png");
}
