#include "CoordinateImage.hpp"
#include <fstream>
#include <iostream>

/**
 * @brief Constructor for CoordinateImage.
 *
 * Initializes a CoordinateImage with the given width and height, allocating
 * a vector of null Coordinate pointers.
 *
 * @param width Width of the image (number of columns).
 * @param height Height of the image (number of rows).
 */
CoordinateImage::CoordinateImage(const int width, const int height)
{
	pixels = std::vector<Coordinate*>(width * height, nullptr);
	this->width = width;
	this->height = height;
}

/**
 * @brief Destructor for CoordinateImage.
 *
 * Cleans up dynamically allocated Coordinate objects in the pixels vector.
 */
CoordinateImage::~CoordinateImage()
{
	auto end = pixels.end();
	for (auto it = pixels.begin(); it != end; ++it){
		if (*it == nullptr) continue; // Nothing to delete
		delete *it;
		*it = nullptr; // Avoid dangling pointer
	}
}

/**
 * @brief Retrieves a modifiable pointer to the pixel at the specified column and row.
 *
 * Throws a logic error if the specified column or row is out of bounds.
 *
 * @param column The x-coordinate of the pixel.
 * @param row The y-coordinate of the pixel.
 * @return Coordinate* const Pointer to the Coordinate object at the specified position.
 * @throws std::logic_error If the column or row is out of bounds.
 */
CoordinateImage::Coordinate * const CoordinateImage::get_pixel(const int column, const int row){
	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.get_pixel"); 

	return pixels[row * width + column];
}

/**
 * @brief Retrieves a constant pointer to the pixel at the specified column and row.
 *
 * Throws a logic error if the specified column or row is out of bounds.
 *
 * @param column The x-coordinate of the pixel.
 * @param row The y-coordinate of the pixel.
 * @return Coordinate* const Constant pointer to the Coordinate object at the specified position.
 * @throws std::logic_error If the column or row is out of bounds.
 */
CoordinateImage::Coordinate * const CoordinateImage::get_pixel(const int column, const int row)const{
	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.get_pixel"); 

	return pixels[row * width + column];
}

/**
 * @brief Sets the pixel at the specified column and row to the given Coordinate object.
 *
 * Throws a logic error if the specified column or row is out of bounds.
 * If no Coordinate exists at the given position, a new one is created.
 *
 * @param column The x-coordinate of the pixel.
 * @param row The y-coordinate of the pixel.
 * @param c The Coordinate object to be set at the specified position.
 * @throws std::logic_error If the column or row is out of bounds.
 */
void CoordinateImage::set_pixel(const int column, const int row, const Coordinate &c){
	
	if (row < 0|| row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.set_pixel"); 

	auto pix = pixels.data() + row * width + column;
 
	if (*pix == nullptr) *pix = new Coordinate();

	(*pix)->x = c.x;
	(*pix)->y = c.y;
	(*pix)->z = c.z;

	(*pix)->color[0] = c.color[0];
	(*pix)->color[1] = c.color[1];
	(*pix)->color[2] = c.color[2];

	(*pix)->xi = c.xi;
	(*pix)->yi = c.yi; 
}

/**
 * @brief Deletes the pixel at the specified column and row.
 *
 * Throws a logic error if the specified column or row is out of bounds.
 * If no Coordinate exists at the given position, nothing happens.
 *
 * @param column The x-coordinate of the pixel.
 * @param row The y-coordinate of the pixel.
 * @throws std::logic_error If the column or row is out of bounds.
 */
void CoordinateImage::delete_pixel(const int column, const int row){

	if (row < 0 || row >= height || column < 0 || column >= width)
		throw std::logic_error("Index out of bounds in CoordinateImage.delete_pixel");

	auto pix = pixels.data() + row*width + column;
	if (*pix == nullptr) return; // Nothing to delete

	delete *pix;
	*pix = nullptr; // Avoid dangling pointer
}

