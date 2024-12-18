#include "CoordinateImage.hpp"

CoordinateImage::CoordinateImage(const int width, const int height)
{
    _pixels = std::vector<std::shared_ptr<Coordinate>>(width * height, nullptr);
    this->_img_width = width;
    this->_img_height = height;
}


CoordinateImage::~CoordinateImage() = default;


CoordinateImage::Coordinate* const CoordinateImage::get_pixel(const int column, const int row)
{
    validateBounds(column, row);
    return _pixels[row * _img_width + column].get();
}


const CoordinateImage::Coordinate* const CoordinateImage::get_pixel(const int column, const int row) const
{
    validateBounds(column, row);
    return _pixels[row * _img_width + column].get();
}


void CoordinateImage::set_pixel(const int column, const int row, const Coordinate& c)
{
    validateBounds(column, row);

    auto& pix = _pixels[row * _img_width + column];
    if (!pix) pix = std::make_unique<Coordinate>();

    pix->x = c.x;
    pix->y = c.y;
    pix->z = c.z;

    pix->color[0] = c.color[0];
    pix->color[1] = c.color[1];
    pix->color[2] = c.color[2];

    pix->xi = c.xi;
    pix->yi = c.yi;
}


void CoordinateImage::delete_pixel(const int column, const int row)
{
    validateBounds(column, row);
    _pixels[row * _img_width + column].reset(); // Resets the unique_ptr, automatically deletes
}


void CoordinateImage::validateBounds(const int column, const int row) const
{
    if (row < 0 || row >= _img_height || column < 0 || column >= _img_width)
        throw std::logic_error("Index out of bounds in CoordinateImage");
}
