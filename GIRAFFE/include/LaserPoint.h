#pragma once

#include <iostream>
#include <cassert>
#include <cmath>
#include <sstream>

/**
 * @class LaserPoint
 * @brief Represents a 3D point with associated ID and color.
 *
 * This class stores a 3D point's coordinates (in the form of a dynamic array),
 * a unique ID for the point, and a color represented by an RGB triplet (red, green, blue).
 * It handles memory management for the dynamic array storing the 3D coordinates.
 */
class LaserPoint {
public:
    /**
     * @brief Default constructor.
     *
     * Initializes the point ID to 0, the 3D coordinates (_xyz) to (0.0, 0.0, 0.0),
     * and the color to black (RGB: {0, 0, 0}).
     */
    LaserPoint()
        : _id(0), _xyz(new double[3] {0.0, 0.0, 0.0}), color{ 0, 0, 0 } { // Initialize color to black
    }

    /**
     * @brief Destructor.
     *
     * Frees the dynamically allocated memory for the 3D coordinates (_xyz) array if
     * it has been allocated, to prevent memory leaks.
     */
    virtual ~LaserPoint() {
        if (_xyz != nullptr)
            delete[] _xyz; // Free the dynamically allocated memory for _xyz array
        _xyz = nullptr; // Set pointer to nullptr to avoid dangling pointer issues
    }

public:

    unsigned int _id; // Unique identifier for the point
    double* _xyz; // Array containing the 3D coordinates of the point (x, y, z)
    unsigned char color[3]; // Color of the point, represented by an RGB triplet (Red, Green, Blue)
};


