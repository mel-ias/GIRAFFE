
#include "PerspectiveImage.h"


PerspectiveImage::PerspectiveImage(DataManager* _dataManager) { 
	// Initialize logging
	logfile = _dataManager->getLogFilePrinter();
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
	_point_loader->set_bb(_data_manager->getBoundingBox()); // Bounding box is specified by DataManager
	_calculator.init_Image(_data_manager->getBoundingBox()); // Initialize image dimensions based on bounding box

	// Read points from binary file and project into image
	_point_loader->read_binary_file(); 
	
	// Process and save the generated image
	_calculator.writeImages();
	_calculator.saveImages();
	_calculator.fill_vectors();
	_calculator.fill_image();
	
	// Set the synthetic image in DataManager for further use
	cv::Mat synthImage = *_calculator.getImage();  
	_data_manager->set_synth_image(synthImage);
}