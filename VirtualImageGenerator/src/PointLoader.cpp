#include "PointLoader.h"

PointLoader::PointLoader(std::string path_pcl, DataManager* data_manager) {
	logfile = data_manager->getLogFilePrinter();
	logfile->append(""); // Add a blank line to log
	logfile->append(TAG + "Initialisation point loader"); // Log initialization message

	_path_point_cloud = path_pcl;
    _input_stream = new std::ifstream(); // Allocate input stream

    // Initialize shift values from DataManager
	_shift_x = data_manager->get_shift_x();
	_shift_y = data_manager->get_shift_y();
	_shift_z = data_manager->get_shift_z();
	logfile->append(TAG + "shift_x " + std::to_string(_shift_x) + ", shift_y " + std::to_string(_shift_y) + ", shift_z " + std::to_string(_shift_z), 3);	 
}


PointLoader::~PointLoader() {
    delete _input_stream;
    _input_stream = 0;
	
}


bool PointLoader::check_filetype_pw() {
	std::size_t loc = _path_point_cloud.find_last_of('.'); // Find the last dot in the filename
	if (_path_point_cloud.size() - loc <= 3) { // Check if there is a valid extension
		return _path_point_cloud.substr(loc + 1, loc + 3) == "pw"; // Return true if extension is ".pw"
	}
	return false;
}


void PointLoader::display_progress_bar(int percent) {
    const int barWidth = 50; // Width of the progress bar
    std::cout << "[";
    int pos = barWidth * percent / 100; // Calculate position based on percentage
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "="; // Filled portion of the bar
        else if (i == pos) std::cout << ">"; // Current position indicator
        else std::cout << " "; // Empty portion of the bar
    }
    std::cout << "] " << std::setw(3) << percent << "%\r"; // Display percentage and reset cursor
    std::cout.flush(); // Ensure immediate output display
}


int PointLoader::read_binary_file() {
    // Check if necessary components are initialized
    if (_imc == nullptr) {
        throw std::logic_error(TAG + "no perspective image found");
    }

    if (_bb == nullptr) {
        throw std::logic_error(TAG + "no bounding box found");
    }

    // Check if file has the correct extension
    if (!check_filetype_pw()) {
        logfile->append(TAG + "input is no pw-file: " + _path_point_cloud);
        return -1;
    }

    // Check if the file path is valid
    if (_path_point_cloud.empty()) {
        logfile->append(TAG + "input has no valid path: " + _path_point_cloud);
        return -1;
    }

    logfile->append(TAG + "load reference point cloud: " + _path_point_cloud);
    logfile->append(TAG + "this may take a while ... ");

    // Open the file in binary mode
    _input_stream->open(_path_point_cloud, std::ifstream::binary);
    if (!_input_stream->good()) {
        logfile->append(TAG + "cannot open file: " + _path_point_cloud);
        return -1;
    }

    // Get the total file size for progress tracking
    _input_stream->seekg(0, std::ios::end);
    std::streampos totalSize = _input_stream->tellg();
    _input_stream->seekg(1, std::ios::beg);  // Skip the first character

    // Retrieve frustum limits for filtering
    double xMax_world = _bb->get_xmax_World();
    double yMax_world = _bb->get_ymax_World();
    double zMax_world = _bb->get_zmax_World();
    double xMin_world = _bb->get_xmin_World();
    double yMin_world = _bb->get_ymin_World();
    double zMin_world = _bb->get_zmin_World();

    const size_t batchSize = 5000;
    std::vector<std::future<void>> futures;
    long long i = 0; // Point counter
    std::streampos currentPos;
    int lastPercent = 0;

    // Print initial progress bar
    display_progress_bar(0);

    // Read and process the binary data in batches
    while (!_input_stream->eof()) {
        std::vector<LaserPoint*> batch;
        batch.reserve(batchSize); // Reserve space to improve performance

        // Read points in the current batch
        for (size_t j = 0; j < batchSize && !_input_stream->eof(); ++j) {
            double x = 0.0, y = 0.0, z = 0.0;
            unsigned char color[3] = { 0, 0, 0 }; // Initialize color array

            _input_stream->read(reinterpret_cast<char*>(&x), sizeof(x));
            _input_stream->read(reinterpret_cast<char*>(&y), sizeof(y));
            _input_stream->read(reinterpret_cast<char*>(&z), sizeof(z));
            _input_stream->read(reinterpret_cast<char*>(&color[0]), 1);
            _input_stream->read(reinterpret_cast<char*>(&color[1]), 1);
            _input_stream->read(reinterpret_cast<char*>(&color[2]), 1);

            // Stop on read error
            if (!_input_stream->good()) {
                break;
            }

            // Apply shifts to point coordinates
            x -= _shift_x;
            y -= _shift_y;
            z -= _shift_z;

            // Filter out points outside the frustum
            if (x > xMax_world || x < xMin_world ||
                y > yMax_world || y < yMin_world ||
                z > zMax_world || z < zMin_world) {
                continue; 
            }

            // Store the point
            LaserPoint* lp = new LaserPoint();
            lp->_id = static_cast<unsigned int>(i);
            lp->_xyz[0] = x;
            lp->_xyz[1] = y;
            lp->_xyz[2] = z;
            lp->color[0] = color[0];
            lp->color[1] = color[1];
            lp->color[2] = color[2];

            batch.push_back(lp); 
            ++i; // Increment point counter
        }

        // Launch processing of the batch asynchronously
        if (!batch.empty()) {
            futures.push_back(std::async(std::launch::async, [this, batch]() {
                for (LaserPoint* lp : batch) {
                    if (lp) {
                        _imc->projectPoint(lp); // Project point in perspective image
                        delete lp; // Clean up memory after processing
                    }
                }
                }));
        }

        // Update the progress bar
        currentPos = _input_stream->tellg();
        int percent = static_cast<int>((static_cast<double>(currentPos) / totalSize) * 100);
        if (percent >= lastPercent + 1) { // Update every 1%
            display_progress_bar(percent);
            lastPercent = percent;
        }
    }

    // Wait for all threads to complete
    for (auto& future : futures) {
        future.get();
    }

    _input_stream->close(); // Close the input file
    display_progress_bar(100); // Finalize progress bar at 100%
    std::cout << std::endl; // New line after progress bar completion
    return 0;
}
