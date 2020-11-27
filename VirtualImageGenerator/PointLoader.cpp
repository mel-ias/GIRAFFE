#include <fstream>
#include <cfloat>
#include <string>
#include "PointLoader.h"

// name from Scan2Image
// C'tor
PointLoader::PointLoader(std::string filename, DataManager* dataManager) {
    // init variables
	//my_imc = nullptr;
	//my_bb = nullptr;
	logfile = dataManager->getLogFilePrinter();
	logfile->append(""); // return one line
	logfile->append(TAG + "----initialisation point loader----"); // return one line

	path_point_cloud = filename;
	
    input_stream_ptr = new std::ifstream();
	color_points = false;

	shifter_x = dataManager->get_utm_shift_x();
	shifter_y = dataManager->get_utm_shift_y();

	logfile->append(TAG + "shifter_x " + std::to_string(shifter_x) + ", shifter_y " + std::to_string(shifter_y), 3);


		 
}

// D'tor
PointLoader::~PointLoader() {
    // tidy up

    delete input_stream_ptr;
    input_stream_ptr = 0;
	
}



void PointLoader::check_color(){
	
	input_stream_ptr->open(path_point_cloud.c_str(), std::ifstream::binary);
	if (!input_stream_ptr->good()){
		logfile->append(TAG + "check color, cannot open filename: " + path_point_cloud);
		return;
	}

	color_points = input_stream_ptr->get() != 0;
	my_imc->WriteColorPoints(color_points);
	input_stream_ptr->close();

}


/*
this reads a binary file (stored informations: id x y z intensity)
In this method are only read the points to the pointcloud, that are in the visible space.
*/

int PointLoader::read_binary_file(){
	
	if (my_imc == nullptr){
		throw std::logic_error(TAG + "no perspective image found"); // programmers fault!
		return -1;
	}

	if (my_bb == nullptr){
		throw std::logic_error(TAG + "no bounding box found"); // programmers fault!
		return -1;
	}

	if (!check_file()){
		logfile->append(TAG + "input is no pw-file: " + path_point_cloud);
		return -1;
	}

	if (path_point_cloud == "") {
		logfile->append(TAG + "input has no valid path: " + path_point_cloud);
		return -1;
	}

	// getting the size of _file
	logfile->append(TAG + "calc file size of reference point cloud: " + path_point_cloud);
	long size_point_cloud = 0;
	size_point_cloud = calculate_file_size(path_point_cloud);
	
	//getFileSize(f_size);
	
	input_stream_ptr->open(path_point_cloud, std::ifstream::binary);
	if (!input_stream_ptr->good()){
		logfile->append(TAG + "cannot open file: " + path_point_cloud);
		return -1;
	}

	input_stream_ptr->get(); // ignore first char

	if (size_point_cloud == 0){
		logfile->append(TAG + "cannot get file size " + path_point_cloud);
	}
	else{
		logfile->append(TAG + "file size of reference point cloud: " + std::to_string(size_point_cloud / 1024 / 1024) + " Mbytes");
		
		size_point_cloud -= 1; // ignore first byte (it says if we have color or not)
		
		// overwrite f_size to the point count in file
		if (color_points) 
			size_point_cloud /= (3 * sizeof(double) + 3 * sizeof(char));
		else 
			size_point_cloud /= (3 * sizeof(double)+sizeof(float));

		logfile->append(TAG + "will read  " + std::to_string(size_point_cloud) + " points ...");

		// now save 1% of the point count (so one . in cout will show 10% from the file)
		size_point_cloud = static_cast<long long>(std::ceil(size_point_cloud* 0.01));
	}

	
	// the world borders from the boundingbox, we needed.
	double xMax_world = my_bb->get_xmax_World();
	double yMax_world = my_bb->get_ymax_World();
	double zMax_world = my_bb->get_zmax_World();
	double xMin_world = my_bb->get_xmin_World();
	double yMin_world = my_bb->get_ymin_World();
	double zMin_world = my_bb->get_zmin_World();



	// read the binary file to the Pointcloud
	
	LaserPoint* lp = new LaserPoint(3);
	unsigned int id = 0u;
	long long i = 0;
	long long p = 0;
	double x, y, z;
	float intensity;
	unsigned char color[3];
	int percent = 0;
	while (!input_stream_ptr->eof()){
		input_stream_ptr->read(reinterpret_cast<char*>(&x), sizeof(x));
		input_stream_ptr->read(reinterpret_cast<char*>(&y), sizeof(y));
		input_stream_ptr->read(reinterpret_cast<char*>(&z), sizeof(z));

		

		if (color_points){
			input_stream_ptr->read(reinterpret_cast<char*>(&color[0]), 1);
			input_stream_ptr->read(reinterpret_cast<char*>(&color[1]), 1);
			input_stream_ptr->read(reinterpret_cast<char*>(&color[2]), 1);
		}
		else input_stream_ptr->read(reinterpret_cast<char*>(&intensity), sizeof(intensity));
		   
		if (size_point_cloud > 0){
			if (p % size_point_cloud == 0){
				++percent;
				std::cout << "\r" << percent << " % " << std::flush; // one and only std::cout in code!
			}
		}
		++p;

		// apply shifter
		x -= shifter_x;
		y -= shifter_y;

		// check if the point is necessary
		if (x > xMax_world || x < xMin_world) continue;
		if (y > yMax_world || y < yMin_world) continue;
		if (z > zMax_world || z < zMin_world) continue;

		// we need this point, so store him.
		lp->_id = id;
		if (color_points){
			lp->color[0] = color[0];
			lp->color[1] = color[1];
			lp->color[2] = color[2];
		}
		else lp->_intensity = intensity;
		lp->_xyz[0] = x;
		lp->_xyz[1] = y;
		lp->_xyz[2] = z;

		++i;
		my_imc->projectPoint(lp);
		
	}
	delete lp;

	input_stream_ptr->close();

	logfile->append("");
	logfile->append(TAG + std::to_string(i) + " of " + std::to_string(size_point_cloud) + " points projected");
	return 0;
}

bool PointLoader::check_file(){
	
	std::size_t loc = path_point_cloud.find_last_of('.');
	if (path_point_cloud.size() - loc <= 3){
		return path_point_cloud.substr(loc+1, loc + 3) == "pw";
	}
	return false;
}

/*
void PointLoader::getFileSize(long long &size){
	std::ifstream temp_ifs;
	temp_ifs.open(_file, std::ifstream::binary);
	if (!temp_ifs.good()) return;
	std::streampos begin = temp_ifs.tellg();
	temp_ifs.seekg(0, temp_ifs.end);
	std::streampos end = temp_ifs.tellg();
	temp_ifs.close();
	size = end - begin;
}*/


// needs #include <fstream>
long PointLoader::calculate_file_size(std::string path) {
	FILE *pFile = NULL;
	fopen_s(&pFile, path.c_str(), "rb"); // get the file stream
	fseek(pFile, 0, SEEK_END); // set the file pointer to end of file
	long Size = ftell(pFile); // get the file size
							 // rewind( pFile ); // return the file pointer to begin of file if you want to read it		
	fclose(pFile); // close stream and release buffer

	return Size;
}
