
#include "PerspectiveImage.h"


PerspectiveImage::PerspectiveImage(DataManager* _dataManager) { 
	// Initialize logging
	logfile = _dataManager->get_logfile();
	logfile->append(""); // Empty line for log formatting
	logfile->append(TAG + "---- initialisation perspective image ----"); // Start log for initialization

	// Initialize variables, including the DataManager and point loader
	_data_manager = _dataManager;
	_calculator.init(_dataManager);

	// Create the PointLoader with the path from DataManager
	_point_loader = new PointLoader(_dataManager->get_path_file_pointcloud().string(), _dataManager);  
}


PerspectiveImage::~PerspectiveImage() {
	// Clean up dynamically allocated point loader
	delete _point_loader;
  
}


void PerspectiveImage::generateImage(){
   	
	// Set up calculator and bounding box for point loader
	_point_loader->set_imc(&_calculator); // Initializes ImCalculator within point loader
	_point_loader->set_bb(_data_manager->get_frustum()); // Bounding box is specified by DataManager
	_calculator.init_image(_data_manager->get_frustum()); // Initialize image dimensions based on bounding box

	auto t0 = std::chrono::high_resolution_clock::now();

	// Read points from binary file and project into image
	_point_loader->read_binary_file(); 
	
	auto t1 = std::chrono::high_resolution_clock::now();

	// Process and save the generated image
	_calculator.write_images();

	auto t2 = std::chrono::high_resolution_clock::now();

	_calculator.save_images();

	auto t3 = std::chrono::high_resolution_clock::now();

	_calculator.fill_vectors();
	_calculator.fill_image();

	auto t4 = std::chrono::high_resolution_clock::now();

	std::cout << "read_binary_file: "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()
		<< " ms\n";

	std::cout << "write_images: "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
		<< " ms\n";

	std::cout << "save_images: "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count()
		<< " ms\n";

	std::cout << "fill_vectors and fill_images: "
		<< std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count()
		<< " ms\n";
	
	// Set the synthetic image in DataManager for further use
	cv::Mat synthImage = *_calculator.get_image();  
	_data_manager->set_synth_image(synthImage);
}