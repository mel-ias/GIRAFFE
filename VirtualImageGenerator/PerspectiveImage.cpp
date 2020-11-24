
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

	_pointloader = new PointLoader(_dataManager->getPathInputPointCloud(), _dataManager); //new PointLoader(path);    

	// Übergebe Outputpfad
	setOutputPath(_dataManager->getPathOutputImage());

}

// D'tor
PerspectiveImage::~PerspectiveImage() {
	// tidy up
	delete _pointloader;
  
}



// checks whether all necessary variables are set
void
PerspectiveImage::checkVariables(){
    if(_outputPath.empty()){
		logfile->append(TAG + "output path undefined. return.");
        exit(0);
    }
}

// generates image
void
PerspectiveImage::generateImage(){
    checkVariables();
	
	// NUR MATCHING TESTEN!
	_pointloader->set_imc(&calculator); //startet init von imcalculator
	_pointloader->check_color();
	
	_pointloader->set_bb(dataManager->getBoundingBox()); // bouding box is specified by dataManager
	calculator.init_Image(dataManager->getBoundingBox()); // bouding box is specified by dataManager

    // read point file
	_pointloader->read_binary_file();
	
	calculator.writeImages();
	calculator.saveImages(_outputPath);
	calculator.fillVektorsForImFill();

	size_t k_for_knn = 3;
	calculator.fillImage(k_for_knn); 

	cv::Mat synthImage = calculator.get_FilledImage_Knn();
	dataManager->setSynthImage(synthImage);
}




// FilterAlgorithmen für Bildvorberarbeitung. aktuell implementiert: WallisFilter, GaussFilter
cv::Mat PerspectiveImage::applyFilterAlgorithms(
	const cv::Mat& imageToBeFiltered,
	PerspectiveImage::FILTER_APPS filterToBeApplied) {

	cv::Mat imageFiltered;
	
	// Apply slight Gaussian blurring
	if (filterToBeApplied == APPLY_GAUSSIAN) {
		logfile->append(TAG + "start Image pre-processing, gaussian blur");
		cv::GaussianBlur(imageToBeFiltered, imageFiltered, cv::Size(5, 5), 8, 8); // Size 5,5, Sigma 8x8 gute kombi
	}

	// Apply Wallis Filtering
	if (filterToBeApplied == APPLY_WALLIS) {
		int k = 3;
		logfile->append(TAG + "start image pre-processing, wallis filter");
		WallisGrayscaleChannelOnly(imageToBeFiltered, imageFiltered, k);
	}

	// Apply Denoising
	if (filterToBeApplied == APPLY_FASTNLMEANS_COLOR) {
		float h = 10; float hColor = 10; int tempWindow = 7; int searchWindow = 21;
		logfile->append(TAG + "start Image pre-processing, denoise image applying fast nl_means");
		cv::fastNlMeansDenoisingColored(imageToBeFiltered, imageFiltered, h, hColor, tempWindow, searchWindow);
	}

	// APPLY Bilateral Filtering
	if (filterToBeApplied == APPLY_BILATERAL_COLOR) {
		int d = 5, borderType = cv::BORDER_DEFAULT; double sigmaColor = 200, sigmaSpace = 200;
		logfile->append(TAG + "start image pre-processing, Ddnoise Image applying bilateral filter");
		cv::bilateralFilter(imageToBeFiltered, imageFiltered, d, sigmaColor, sigmaSpace, borderType); // sigma > 150 -> cartoon effekt... bei beiden testen?
	}

	return imageFiltered;
}
