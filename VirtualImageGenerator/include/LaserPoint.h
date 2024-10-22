#pragma once

#include <iostream>
#include <cassert>
#include <cmath>
#include <sstream>


class LaserPoint {
public:
    // Constructor
    LaserPoint()
        : _id(0), _xyz(new double[3] {0.0, 0.0, 0.0}), color{ 0, 0, 0 } { // Initialize color to black
    }

    // Destructor
    virtual ~LaserPoint() {
        if (_xyz != nullptr)
            delete[] _xyz;
        _xyz = nullptr;
    }

public:
    // Point id
    unsigned int _id;
    // Array with 3D coordinates
    double* _xyz;

    // 3 channel color (red, green, blue)
    unsigned char color[3];
};


