#include <fstream>
#include <cfloat>
#include <string>
#include "PointLoader.h"


// C'tor
PointLoader::PointLoader(std::string filename, DataManager* dataManager) {
    // init variables
	//my_imc = nullptr;
	//my_bb = nullptr;
	logfile = dataManager->getLogFilePrinter();
	logfile->append(""); // return one line
	logfile->append(TAG + "Initialisation point loader"); // return one line

	path_point_cloud = filename;
	
    input_stream_ptr = new std::ifstream();

	shift_x = dataManager->get_shift_x();
	shift_y = dataManager->get_shift_y();
	shift_z = dataManager->get_shift_z();
	logfile->append(TAG + "shift_x " + std::to_string(shift_x) + ", shift_y " + std::to_string(shift_y) + ", shift_z " + std::to_string(shift_z), 3);	 
}

// D'tor
PointLoader::~PointLoader() {
    // tidy up

    delete input_stream_ptr;
    input_stream_ptr = 0;
	
}


bool PointLoader::check_file() {

	std::size_t loc = path_point_cloud.find_last_of('.');
	if (path_point_cloud.size() - loc <= 3) {
		return path_point_cloud.substr(loc + 1, loc + 3) == "pw";
	}
	return false;
}



void PointLoader::display_progress_bar(int percent) {
    const int barWidth = 50; // Adjust the width of the progress bar
    std::cout << "[";
    int pos = barWidth * percent / 100;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << std::setw(3) << percent << "%\r"; // Print percentage and move cursor back
    std::cout.flush(); // Ensure output is flushed immediately
}

int PointLoader::read_binary_file() {
    if (my_imc == nullptr) {
        throw std::logic_error(TAG + "no perspective image found");
    }

    if (my_bb == nullptr) {
        throw std::logic_error(TAG + "no bounding box found");
    }

    if (!check_file()) {
        logfile->append(TAG + "input is no pw-file: " + path_point_cloud);
        return -1;
    }

    if (path_point_cloud.empty()) {
        logfile->append(TAG + "input has no valid path: " + path_point_cloud);
        return -1;
    }

    logfile->append(TAG + "load reference point cloud: " + path_point_cloud);
    logfile->append(TAG + "this may take a while ... ");

    input_stream_ptr->open(path_point_cloud, std::ifstream::binary);
    if (!input_stream_ptr->good()) {
        logfile->append(TAG + "cannot open file: " + path_point_cloud);
        return -1;
    }

    // Get the total size of the file
    input_stream_ptr->seekg(0, std::ios::end);
    std::streampos totalSize = input_stream_ptr->tellg();
    input_stream_ptr->seekg(1, std::ios::beg);  // Skip the first character

    // Getting bounding box limits
    double xMax_world = my_bb->get_xmax_World();
    double yMax_world = my_bb->get_ymax_World();
    double zMax_world = my_bb->get_zmax_World();
    double xMin_world = my_bb->get_xmin_World();
    double yMin_world = my_bb->get_ymin_World();
    double zMin_world = my_bb->get_zmin_World();

    const size_t batchSize = 5000;
    std::vector<std::future<void>> futures;
    long long i = 0; // Point counter
    std::streampos currentPos;
    int lastPercent = 0;

    // Print initial progress bar
    display_progress_bar(0);

    while (!input_stream_ptr->eof()) {
        // Create a vector to hold the batch of points
        std::vector<LaserPoint*> batch;
        batch.reserve(batchSize); // Reserve space to improve performance

        // Read a batch of points
        for (size_t j = 0; j < batchSize && !input_stream_ptr->eof(); ++j) {
            double x, y, z;
            unsigned char color[3];

            input_stream_ptr->read(reinterpret_cast<char*>(&x), sizeof(x));
            input_stream_ptr->read(reinterpret_cast<char*>(&y), sizeof(y));
            input_stream_ptr->read(reinterpret_cast<char*>(&z), sizeof(z));
            input_stream_ptr->read(reinterpret_cast<char*>(&color[0]), 1);
            input_stream_ptr->read(reinterpret_cast<char*>(&color[1]), 1);
            input_stream_ptr->read(reinterpret_cast<char*>(&color[2]), 1);

            // Check if the read was successful
            if (!input_stream_ptr->good()) {
                //logfile->append(TAG + "Error reading data from file.");
                break; // Exit loop on error
            }

            // Apply shift
            x -= shift_x;
            y -= shift_y;
            z -= shift_z;

            // Check if the point is necessary
            if (x > xMax_world || x < xMin_world ||
                y > yMax_world || y < yMin_world ||
                z > zMax_world || z < zMin_world) {
                continue; // Skip unnecessary points
            }

            // Create and store the LaserPoint object
            LaserPoint* lp = new LaserPoint();
            lp->_id = static_cast<unsigned int>(i);
            lp->_xyz[0] = x;
            lp->_xyz[1] = y;
            lp->_xyz[2] = z;
            lp->color[0] = color[0];
            lp->color[1] = color[1];
            lp->color[2] = color[2];

            batch.push_back(lp); // Store the pointer in the batch
            ++i; // Increment the point counter
        }

        // Only launch if we have valid points
        if (!batch.empty()) {
            futures.push_back(std::async(std::launch::async, [this, batch]() {
                for (LaserPoint* lp : batch) {
                    if (lp) { // Check if the pointer is valid
                        my_imc->projectPoint(lp);
                        delete lp; // Clean up after processing
                    }
                }
                }));
        }

        // Track the current position in the file for progress
        currentPos = input_stream_ptr->tellg();
        int percent = static_cast<int>((static_cast<double>(currentPos) / totalSize) * 100);
        if (percent >= lastPercent + 1) { // Update every 1%
            display_progress_bar(percent);
            lastPercent = percent;
        }
    }

    // Wait for all threads to finish
    for (auto& future : futures) {
        future.get();
    }

    input_stream_ptr->close();
    display_progress_bar(100); // Final progress at 100%
    std::cout << std::endl; // Move to the next line after the progress bar is done
    return 0;
}
