#ifndef COORDINATE_IMAGE_HPP
#define COORDINATE_IMAGE_HPP

#include <vector>
#include <memory> // For std::unique_ptr
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include "opencv2\opencv.hpp"

/**
 * @brief Class used to map the coordinates of each point from the point cloud to the corresponding pixel in the virtual image.
 *
 * In this project, the class is primarily used to read and write data.
 */
class CoordinateImage
{
public:

    /**
     * @brief Structure representing a pixel with 3D coordinates and color information.
     */
    struct Coordinate
    {
        double x, y, z;    ///< 3D coordinates of the pixel.
        unsigned char color[3]; ///< RGB color values.
        double xi, yi;      ///< Additional coordinates for image alignment.

        /**
         * @brief Default constructor initializing all values to 0.
         */
        Coordinate()
            : x(0), y(0), z(0), xi(0), yi(0)
        {
            color[0] = 0;
            color[1] = 0;
            color[2] = 0;
        }

        /**
         * @brief Parameterized constructor for initializing with specific values.
         *
         * @param xc X-coordinate.
         * @param yc Y-coordinate.
         * @param zc Z-coordinate.
         * @param colorc Pointer to an OpenCV color structure.
         * @param xic Additional coordinate.
         * @param yic Additional coordinate.
         */
        Coordinate(double xc, double yc, double zc, cv::Point3_<uchar>* colorc, double xic, double yic)
            : x(xc), y(yc), z(zc), xi(xic), yi(yic)
        {
            color[0] = colorc->x;
            color[1] = colorc->y;
            color[2] = colorc->z;
        }
    };

    // Disable the default constructor
    CoordinateImage() = delete;

    /**
    * @brief Constructor for CoordinateImage.
    *
    * Initializes a CoordinateImage with the given width and height, allocating
    * a vector of nullptrs (initially no Coordinates).
    *
    * @param width Width of the image (number of columns).
    * @param height Height of the image (number of rows).
    */
    CoordinateImage(const int width, const int height);


    /**
     * @brief Destructor for CoordinateImage.
     *
     * Automatically handled by shared_ptr, no manual deletion required.
     */
    ~CoordinateImage();


    // Disable copy constructor and assignment operator
    CoordinateImage(CoordinateImage& src) = delete;
    CoordinateImage& operator=(CoordinateImage& rhs) = delete;


    /**
    * @brief Retrieves a modifiable pointer to the pixel at the specified column and row.
    *
    * Throws a logic error if the specified column or row is out of bounds.
    *
    * @param column The x-coordinate of the pixel.
    * @param row The y-coordinate of the pixel.
    * @return Coordinate* Pointer to the Coordinate object at the specified position.
    * @throws std::logic_error If the column or row is out of bounds.
    */
    Coordinate* const get_pixel(const int column, const int row);


    /**
    * @brief Retrieves a constant pointer to the pixel at the specified column and row.
    *
    * Throws a logic error if the specified column or row is out of bounds.
    *
    * @param column The x-coordinate of the pixel.
    * @param row The y-coordinate of the pixel.
    * @return const Coordinate* Constant pointer to the Coordinate object at the specified position.
    * @throws std::logic_error If the column or row is out of bounds.
    */
    const Coordinate* const get_pixel(const int column, const int row) const;


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
    void set_pixel(const int column, const int row, const Coordinate& c);



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
    void delete_pixel(const int column, const int row);


    /**
     * @brief Returns the width of the image.
     *
     * @return int The width of the image.
     */
    int getWidth() const { return width; }


    /**
     * @brief Returns the height of the image.
     *
     * @return int The height of the image.
     */
    int getHeight() const { return height; }


    /**
     * @brief Returns a copy of the vector of pixel pointers.
     *
     * @return std::vector<std::shared_ptr<Coordinate>> Vector of unique pointers to Coordinate.
     */
    std::vector<std::shared_ptr<Coordinate>> getPixels() const { return pixels; }

private:
    int width, height; // Image dimensions
    std::vector<std::shared_ptr<Coordinate>> pixels; // Vector storing unique pointers to Coordinates

    std::stringstream logfile; // Logfile stream

    /**
     * @brief Validates that the given column and row are within bounds.
     *
     * Throws a logic error if the specified column or row is out of bounds.
     *
     * @param column The x-coordinate of the pixel.
     * @param row The y-coordinate of the pixel.
     * @throws std::logic_error If the column or row is out of bounds.
     */
    void validateBounds(const int column, const int row) const;
};

#endif // COORDINATE_IMAGE_HPP
