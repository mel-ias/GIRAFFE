#include "CoordinateImage.hpp"
#include<fstream>
#include<iostream>


CoordinateImage::CoordinateImage(const int width, const int height)
{
	pixels = std::vector<Coordinate*>(width*height, nullptr);
	this->width = width;
	this->height = height;
}


CoordinateImage::~CoordinateImage()
{
	// delete all coordinates.

	auto end = pixels.end();
	for (auto it = pixels.begin(); it != end; ++it){
		if (*it == nullptr) continue; // nothing to delete
		delete *it;
		*it = nullptr;
	}

}

CoordinateImage::Coordinate * const CoordinateImage::getPixel(const int column, const int row){
	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.getPixel"); // programmers fault!

	return pixels[row*width + column];
}

CoordinateImage::Coordinate * const CoordinateImage::getPixel(const int column, const int row)const{
	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.getPixel"); // programmers fault!

	return pixels[row*width + column];
}


void CoordinateImage::setPixel(const int column, const int row, const Coordinate &c){
	
	if (row < 0|| row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.setPixel"); // programmers fault!

	/* pix: 
		pointer zum pointer der Daten (**) 
		zugriff auf datenpointer: *pix (Coordinate*)
		Zugriff auf Daten : **pix (coordinate) oder (*pix)->x (coordinate.x)
		Dieser umständliche Weg muss gegangen werden, damit sich *pix auf die Daten innerhalb des Vectorpixels bezieht. 
		*pix = nullptr löscht also den Pointer in den Vectordaten.
	*/
	auto pix = pixels.data() + row*width + column;
 

	if (*pix == nullptr) *pix = new Coordinate();

	(*pix)->x = c.x;
	(*pix)->y = c.y;
	(*pix)->z = c.z;

	(*pix)->color[0] = c.color[0];
	(*pix)->color[1] = c.color[1];
	(*pix)->color[2] = c.color[2];
	

	(*pix)->intensity = c.intensity;

	(*pix)->xi = c.xi;
	(*pix)->yi = c.yi; 

	//std::cout << "Set Pixel: " << (*pix)->x << "," << (*pix)->y << "," << (*pix)->z << "," 
	//	<< static_cast<int>((*pix)->color[0]) << "," << static_cast<int>((*pix)->color[1]) << "," << static_cast<int>((*pix)->color[2]) << "," << std::endl;
}

void CoordinateImage::deletePixel(const int column, const int row){

	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.deletePixel"); // programmers fault!

	auto pix = pixels.data() + row*width + column;
	if (*pix == nullptr) return; // nothing to delete

	delete *pix;
	*pix = nullptr;
}



/*

// Schreibe CoordImage in Datei raus


//std::ofstream myfile;
//myfile.open("cim_out.txt");

std::cout << "Pixel: " << 0 << "," << 0
<< ", Value: "
<< cImL.getPixel(0,0)->x << ", " << cImL.getPixel(0,0)->y << ", " << cImL.getPixel(0,0)->z
<< std::endl;


/*for (int r = 0; r < cImL.getHeight(); ++r) {
for (int c = 0; c < cImL.getWidth(); ++c) {

if (cImL.getPixel(c, r)->x != 0 && cImL.getPixel(c, r)->y != 0 && cImL.getPixel(c, r)->z != 0)
std::cout << "Pixel: " << r << "," << c
<< ", Value: "
<< cImL.getPixel(c, r)->x << ", " << cImL.getPixel(c, r)->y << ", " << cImL.getPixel(c, r)->z
<< std::endl;
}
}*/
//myfile.close();

