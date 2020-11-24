
#pragma once

#ifndef spectralindices_h
#define spectralindices_h


#include "DataManager.h"
#include "LogfilePrinter.h"

#endif

/*
#' RGB indices
#'
#' @description
#' This function calculates various spectral indices from a RGB. It returns at least red green and blue as splitted channels in a stack. Additionally you can choose RGB indices.
#' \code{Raster*} object.
#'
#' @param rgb a \code{RasterStack} or \code{RasterBrick} object. 3
#' bands are mandatory (for RGB indices they should be: "red", "green" and "blue").
#' @param rgbi the implemented RGB indices currently \link{seealso}


#' A \code{RasterLayer} with the index calculates as:\cr
#' BI  sqrt((R**2+G**2+B*2)/3 Brightness Index\cr
#' CI (R-G)/(R+G) Soil Colour Index\cr
#' GLI (2*g - r - b)/(2*g + r + b) Green leaf index Vis Louhaichi et al. (2001)\cr
#' HI (2*R-G-B)/(G-B) Primary colours Hue Index\cr
#' NDTI (R-G)/(R+G) Normalized difference turbidity index Water\cr
#' NGRDI (G-R)/(G+R) Normalized green red difference index (sometimes GRVI) Tucker (1979)
#' RI R**2/(B*G**3) Redness Index\cr
#' SI (R-B)/(R+B) Spectral Slope Saturation Index\cr
#' TGI  -0.5[190(R670-R550)-120(R670 - R480)] The triangular greenness index (TGI) estimates chlorophyll concentration in leaves and canopies\cr
#' VARI (green-red)/(green+red-blue). A Visible Atmospherically Resistant Index (VARI)\cr
#' VVI  (1-(r-30)/(r+30))*(1-(g-50)/(g+50))*(1-(b-1)/(b+1))
#'
#'
#' @name rs_rgbIndices
#' @export rs_rgbIndices
#'
#' @references
#'
#' Planetary Habitability Laboratory (2015): Visible Vegetation Index (VVI). Available online via \url{http://phl.upr.edu/projects/visible-vegetation-index-vvi}.
#'
#' Lacaux, J. P., Tourre, Y. M., Vignolles, C., Ndione, J. A., and Lafaye, M.: Classification of ponds from high-spatial resolution remote sensing: Application to Rift Valley Fever epidemics in Senegal, Remote Sens. Environ., 106, 66-74, 2007.
#'
#' Gitelson, A., et al.: Vegetation and Soil Lines in Visible Spectral Space: A Concept and Technique for Remote Estimation of Vegetation Fraction.  International Journal of Remote Sensing 23 (2002): 2537-2562. (VARI)
#'
#' MADEIRA, J., BEDIDI, A., CERVELLE, B., POUGET, M. and FLAY, N., 1997, Visible spectrometric indices of hematite (Hm) and goethite (Gt) content in lateritic soils: 5490 N. Levin et al. the application of a Thematic Mapper (TM) image for soil-mapping in Brasilia, Brazil. International Journal of Remote Sensing, 18, pp. 2835-2852.
#'
#' MATHIEU, R., POUGET, M., CERVELLE, B. and ESCADAFAL, R., 1998, Relationships between satellite-based radiometric indices simulated using laboratory reflectance data and typic soil colour of an arid environment. Remote Sensing of Environment, 66, pp. 17-28.
#'
#' Louhaichi, M., Borman, M.M., Johnson, D.E., 2001. Spatially located platform and aerial photography for documentation of grazing impacts on wheat. Geocarto International 16, 65-70.
#'
#' Tucker, C.J., 1979. Red and photographic infrared linear combinations for monitoring vegetation. Remote Sensing of Environment 8, 127-150.
#'
#' @seealso
#' For a comprehensive overview of remote sensing indices have a look at: \url{http://www.indexdatabase.de/db/i.php}(A database for remote sensing indices)\cr
#' Wavelength ranges for overlapping digital camera bands are: red 580-670 nm, green 480-610 nm, and blue 400-520 nm (Hunt et al., 2005)
#' http://digitalcommons.unl.edu/cgi/viewcontent.cgi?article=2161&context=usdaarsfacpub
#'
#' @examples
#' \notrun{
#' library(raster)
#' url <- "https://upload.wikimedia.org/wikipedia/commons/2/28/RGB_illumination.jpg"
#' dFile <- download.file(url, "Image.jpg")
#' img <- stack("Image.jpg")
#' plotRGB(img)
#' rgbi <- rgbI(img)
#' plot(rgbI, col = gray(255:0/255))
#' }
#'
#'

QUELLE:: https://github.com/gisma/uavRst/blob/master/R/rs_rgbi.R
*/



/*
//https://gist.github.com/squarethecircle/98e9cf54278be3a8d072
inline cv::Mat VVI_2(LogFile* logfile, cv::Mat& image) {

	const std::string TAG = "Vegetation:VVI2:\t\t";
	for (int i = 0; i < image.cols; i++) {
		for (int j = 0; j < image.rows; j++) {
			cv::Vec3b &intensity = image.at<cv::Vec3b>(j, i);
			// calculate pixValue

			double b = (double)intensity.val[0] + 10;
			double g = (double)intensity.val[1] + 10;
			double r = (double)intensity.val[2] + 10;
			double vvi = (1 - abs((r - 40) / (r + 40)))*(1 - abs((g - 60) / (g + 60)))*(1 - abs((b - 10) / (b + 10)));
			
			//colormap
			double a = (1 - 4 * vvi) / 0.25;	//invert and group
			int X = (int)a;	//this is the integer part
			uchar Y = (uchar)255 * (a - X); //fractional part from 0 to 255
			uchar r, g, b;
			switch (X)
			{
			case 0: r = 255; g = Y; b = 0; break;
			case 1: r = 255 - Y; g = 255; b = 0; break;
			case 2: r = 0; g = 255; b = Y; break;
			case 3: r = 0; g = 255 - Y; b = 255; break;
			case 4: r = 0; g = 0; b = 255; break;
			}
			
			cv::Vec3b result;
			result.val[0] = b;
			result.val[1] = g;
			result.val[2] = r;
	
			intensity.val[0] = result.val[0];
			intensity.val[1] = result.val[1];
			intensity.val[2] = result.val[2];

		}
	}
	cv::imwrite("photo_out.jpg", image);

}*/







//## calculate Visible Vegetation Index vvi
inline void VVI(LogFile* logfile, const cv::Mat rgbImage, std::string fileName) {

	const std::string TAG = "Vegetation:VVI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);


	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {
	 
	}
	else {
		logfile->append(TAG + "calculate Visible Vegetation Index (VVI)");

		cv::Mat result_red, result_green, result_blue, result; 

		//VVI  (1-(r-30)/(r+30))*(1-(g-50)/(g+50))*(1-(b-1)/(b+1))
		result_red = 1 - (red - 30) / (red + 30);
		result_green = 1 - (green - 50) / (green + 50);
		result_blue = 1 - (blue - 1) / (blue + 1);


		cv::Mat temp;
		cv::multiply(result_red, result_green, temp);
		cv::multiply(temp, result_blue, result);

		//cv::multiply(result_red, result_green, result_green);
		//cv::multiply(result_green, result_blue, result_blue);

		//Schreibe Indexbild raus
		//cv_32FC1_16FC1_image2Txt(result_blue, "VVI"); //error

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result, &min, &max);
		
		scale = 255 / (max - min); // Histogram Equalization

		result.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
		cv::imwrite(fileName + ".png", falseColorsMap);


		// erstelle maske via thresholding
		cv::Mat result_8bit;
		result.convertTo(result_8bit, CV_8UC1);
		
		result_8bit.setTo(1, result_8bit > 0);

		//invertiere für maske
		// initialize the output matrix with zeros
		cv::Mat result_8bit_inv = cv::Mat::zeros(result_8bit.size(), result_8bit.type());

		// create a matrix with all elements equal to 255 for subtraction
		cv::Mat sub_mat = cv::Mat::ones(result_8bit.size(), result_8bit.type()) * 1;

		//subtract the original matrix by sub_mat to give the negative output new_image
		subtract(sub_mat, result_8bit, result_8bit_inv);

		cv::cvtColor(result_8bit_inv, result_8bit_inv, CV_GRAY2RGB);
		cv::Mat rgb_filtered;
		cv::multiply(result_8bit_inv, rgbImage, rgb_filtered);
	
		cv::imwrite(fileName + "_8bit.png", rgb_filtered);
		
		//return result_8bit_inv;
		

		
		
	}
}


inline cv::Mat VARI(LogFile* logfile, const cv::Mat& rgbImage) {

	const std::string TAG = "Vegetation:VARI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);

	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "Visible Atmospherically Resistant Index(VARI)");

		cv::Mat result_red = cv::Mat::zeros(red.size(), CV_32FC1);
		cv::Mat result_green = cv::Mat::zeros(green.size(), CV_32FC1);
		cv::Mat result_blue = cv::Mat::zeros(blue.size(), CV_32FC1);

		result_red = (green - red);
		result_green = (green + red - blue);

		cv::divide(result_red, result_green, result_blue);

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result_blue, &min, &max);
		
		scale = 255 / (max - min); // Histogram Equalization

		result_blue.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		cv::imwrite("VARI.png", falseColorsMap);



		// erstelle maske via thresholding
		cv::Mat result_8bit;
		result_blue.convertTo(result_8bit, CV_8UC1);

		result_8bit.setTo(1, result_8bit > 0);

		//invertiere für maske
		// initialize the output matrix with zeros
		cv::Mat result_8bit_inv = cv::Mat::zeros(result_8bit.size(), result_8bit.type());

		// create a matrix with all elements equal to 255 for subtraction
		cv::Mat sub_mat = cv::Mat::ones(result_8bit.size(), result_8bit.type()) * 1;

		//subtract the original matrix by sub_mat to give the negative output new_image
		subtract(sub_mat, result_8bit, result_8bit_inv);

		cv::cvtColor(result_8bit_inv, result_8bit_inv, CV_GRAY2RGB);
		cv::Mat rgb_filtered;
		cv::multiply(result_8bit_inv, rgbImage, rgb_filtered);

		cv::imwrite("VARI_8bit.png", rgb_filtered);


		return result_blue;
	}
}



inline cv::Mat NDTI(LogFile* logfile, const cv::Mat& rgbImage) {

	const std::string TAG = "Vegetation:NDTI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);


	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "Normalized difference turbidity index");

		cv::Mat result_red = cv::Mat::zeros(red.size(), CV_32FC1);
		cv::Mat result_green = cv::Mat::zeros(green.size(), CV_32FC1);
		cv::Mat result_blue = cv::Mat::zeros(blue.size(), CV_32FC1);


		result_red = (red - green);
		result_green = (red + green);

		cv::divide(result_red, result_green, result_blue);

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result_blue, &min, &max);
		//std::cout << "colorMapping, " "NDTI" << ", min: " << min << ", max: " << max << std::endl;

		scale = 255 / (max - min); // Histogram Equalization

		result_blue.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		cv::imwrite("NDTI.png", falseColorsMap);

		return result_blue;
	}
}

inline cv::Mat RI(LogFile* logfile, const cv::Mat& rgbImage) {

	const std::string TAG = "Vegetation:RI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);


	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "redness index");

		cv::Mat result_red = cv::Mat::zeros(red.size(), CV_32FC1);
		cv::Mat result_green = cv::Mat::zeros(green.size(), CV_32FC1);
		cv::Mat result_blue = cv::Mat::zeros(blue.size(), CV_32FC1);


		cv::pow(red, 2, result_red);
		cv::pow(green, 3, result_green);
		cv::multiply(blue, result_green, result_green);

		cv::divide(result_red, result_green, result_blue);

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result_blue, &min, &max);
		//std::cout << "colorMapping, " "RI" << ", min: " << min << ", max: " << max << std::endl;

		scale = 255 / (max - min); // Histogram Equalization

		result_blue.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		cv::imwrite("RI.png", falseColorsMap);


		return result_blue;
	}
}



inline cv::Mat CI(LogFile* logfile, const cv::Mat& rgbImage) {

	const std::string TAG = "Vegetation:CI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);


	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "CI Soil Colour Index");

		cv::Mat result_red = cv::Mat::zeros(red.size(), CV_32FC1);
		cv::Mat result_green = cv::Mat::zeros(green.size(), CV_32FC1);
		cv::Mat result_blue = cv::Mat::zeros(blue.size(), CV_32FC1);

		result_red = red - green;
		result_green = red + green;
		cv::divide(result_red, result_green, result_blue);

		
		cv::imwrite("CI_result_blue.png", result_blue);

		//cv::divide(result_red, result_green, result_blue);

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result_blue, &min, &max);
		//std::cout << "colorMapping, " "CI" << ", min: " << min << ", max: " << max << std::endl;

		scale = 255 / (max - min); // Histogram Equalization

		result_blue.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		

		return result_blue;
	}
}


inline cv::Mat BI(LogFile* logfile, const cv::Mat& rgbImage) {
	
	const std::string TAG = "Vegetation:BI:\t\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);

	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "BI Brightness Index");

		cv::Mat result_red = cv::Mat::zeros(red.size(), CV_32FC1);
		cv::Mat result_green = cv::Mat::zeros(green.size(), CV_32FC1);
		cv::Mat result_blue = cv::Mat::zeros(blue.size(), CV_32FC1);

		cv::pow(red, 2, red);
		cv::pow(green, 2, green);

		result_red = (red + green + blue * 2) / 3;
		cv::sqrt(result_red, result_blue);


		//BI <-sqrt((red**2 + green**2 + blue * 2) / 3)


		cv::divide(result_red, result_green, result_blue);

		cv::Mat adjMap;
		double min, max, scale;
		cv::minMaxIdx(result_blue, &min, &max);
		//std::cout << "colorMapping, " "CI" << ", min: " << min << ", max: " << max << std::endl;

		scale = 255 / (max - min); // Histogram Equalization

		result_blue.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		cv::imwrite("CI.png", falseColorsMap);

		return result_blue;
	}
}





//https://agribotix.com/blog/2017/04/30/comparing-rgb-based-vegetation-indices-with-ndvi-for-agricultural-drone-imagery/
inline cv::Mat TGI(LogFile* logfile, const cv::Mat& imageForVegetationMasking, std::string path) {
	const std::string TAG = "Vegetation:TGI:\t";
	cv::Mat bgr[3];   //destination array for splitted channels
	cv::Mat red, blue, green;
	cv::Mat rgbImage;
	imageForVegetationMasking.copyTo(rgbImage);

	split(rgbImage, bgr);//split source  
						 //Note: OpenCV uses BGR color order

	red = bgr[2];	red.convertTo(red, CV_32FC1);
	green = bgr[1]; green.convertTo(green, CV_32FC1);
	blue = bgr[0];	blue.convertTo(blue, CV_32FC1);

	if (red.total() == 0 || blue.total() == 0 || green.total() == 0) {

	}
	else {
		logfile->append(TAG + "TGI Triangular Greeness Index");

		// TGI Grün - 0.39 * Rot - 0.61 * Blau

		cv::Mat result;
		result = green - 0.39 * red - 0.61 * blue;
		
		// Werte über 0 repräsentieren Vegetation --> nutze für Maske!
		cv::Mat maskTGI;
		result.copyTo(maskTGI);

			
		double min, max, scale;
		cv::minMaxIdx(result, &min, &max);
		double mw = (max - min) / 2.0;
		//std::cout << "colorMapping, " "TGI" << ", min: " << min << ", max: " << max << ", mw: " << mw << std::endl;

		maskTGI.setTo(255, maskTGI > 10);
	
		maskTGI.convertTo(maskTGI, CV_8UC1);
		
		maskTGI = 255 - maskTGI; //mache inverses draus
		//maskTGI.convertTo(maskTGI, CV_8UC3);

		// nur Output
		cv::Mat adjMap;
		scale = 255 / (max - min); // Histogram Equalization

		result.convertTo(adjMap, CV_8UC1, scale, -min*scale);

		cv::Mat falseColorsMap;
		cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);

		cv::imwrite(path + "_TGI.png", falseColorsMap);

		cv::Mat copyImage;
		imageForVegetationMasking.copyTo(copyImage, maskTGI);
		cv::imwrite(path + "_mask_TGI.png", copyImage);
	

		return maskTGI;

	}
}




/*
cv::Mat SI(); // SI Spectra Slope Saturation Index
cv::Mat HI(); // HI Primary colours Hue Index
cv::Mat TGI(); // Triangular greenness index
cv::Mat GLI(); // Green leaf index (GLI)
cv::Mat NGRDI(); // NGRDI Normalized green red difference index
cv::Mat GLAI(); // NGRDI Normalized green red difference index

}
else if (item == "BI") {
#  Brightness Index
cat("\ncalculate Brightness Index (BI)")
BI <-sqrt((red**2 + green**2 + blue * 2) / 3)
names(BI) <-"BI"
return(BI)

}
else if (item == "SI") {
# SI Spectra Slope Saturation Index
cat("\ncalculate Spectra Slope Saturation Index (SI)")
SI <-(red - blue) / (red + blue)
names(SI) <-"SI"
return(SI)

}
else if (item == "HI") {
# HI Primary colours Hue Index
cat("\ncalculate Primary colours Hue Index (HI)")
HI<-(2 * red - green - blue) / (green - blue)
names(HI) <-"HI"
return(HI)

}
else if (item == "TGI") {
# Triangular greenness index
cat("\ncalculate Triangular greenness index (TGI)")
TGI <--0.5*(190 * (red - green) - 120 * (red - blue))
names(TGI) <-"TGI"
return(TGI)

}
else if (item == "GLI") {
cat("\ncalculate Green leaf index (GLI)")
# Green leaf index
GLI<-(2 * green - red - blue) / (2 * green + red + blue)
names(GLI) <-"GLI"
return(GLI)

}
else if (item == "NGRDI") {
# NGRDI Normalized green red difference index
cat("\ncalculate Normalized green red difference index  (NGRDI)")
NGRDI<-(green - red) / (green + red)
names(NGRDI) <-"NGRDI"
return(NGRDI)

}
else if (item == "GLAI") {
# NGRDI Normalized green red difference index
cat("\ncalculate greenish Leaf Area Index  (GLAI) (highly experimental)")
vevi<-(green - red) / (green + red - blue)
GLAI = (25 * vevi + 1.25)
names(GLAI) <-"GLAI"
return(GLAI)

}





}; */


