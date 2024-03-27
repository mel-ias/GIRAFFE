
#include "PerspectiveImage.h"

//#include"DataManager.h"

// code on the base from Scan2Image
// C'tor

// Initialisierung mit DatenManager
PerspectiveImage::PerspectiveImage(DataManager* _dataManager) { 
    // init log
	logfile = _dataManager->getLogFilePrinter();
	logfile->append(""); // return one line
	logfile->append(TAG + "---- initialisation perspective image ----"); // return one line

	// init variables (dataManager, pointloader)
	dataManager = _dataManager;
	calculator.init(_dataManager);

	pointloader = new PointLoader(_dataManager->get_path_file_pointcloud(), _dataManager); //new PointLoader(path);    

	
}

// D'tor
PerspectiveImage::~PerspectiveImage() {
	// tidy up
	delete pointloader;
  
}



// generates image
void
PerspectiveImage::generateImage(){
   	
	// NUR MATCHING TESTEN!
	pointloader->set_imc(&calculator); //startet init von imcalculator
	pointloader->check_color();
	
	pointloader->set_bb(dataManager->getBoundingBox()); // bouding box is specified by dataManager
	calculator.init_Image(dataManager->getBoundingBox()); // bouding box is specified by dataManager

    // read point file
	pointloader->read_binary_file();
	
	calculator.writeImages();
	calculator.saveImages();
	calculator.fillVektorsForImFill();

	size_t k_for_knn = dataManager->getKnn(); 
	calculator.fillImage(k_for_knn); 

	cv::Mat synthImage = calculator.get_FilledImage_Knn();
	dataManager->set_synth_image(synthImage);
}