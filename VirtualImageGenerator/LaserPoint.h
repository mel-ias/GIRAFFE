#pragma once

#include <iostream>
#include <cassert>
#include <cmath>
#include <sstream>


class LaserPoint {
public:
    // C'tor
    LaserPoint();
    // C'tor
    LaserPoint(int v);
    // Copy C'tor
    LaserPoint(const LaserPoint& orig);
    // D'tor
    virtual ~LaserPoint();

    // returns string with alle variables
    std::string toString(){
        std::stringstream tmp;
        if(_xyz != 0)
            tmp << "(" << _xyz[0] <<", "<< _xyz[1] <<", "<< _xyz[2] << ")";
        if(_tp != 0)
            tmp << "(" << _tp[0] <<", "<< _tp[1] <<")";
        if(_range != 0)
            tmp << " r:"<< _range << " ";
        if(_intensity != 0)
            tmp << " i:" << _intensity << "";
		if (color != 0)
			tmp << " color:" << static_cast<unsigned int>(color[0]) << ", " << static_cast<unsigned int>(color[1]) << ", " << static_cast<unsigned int>(color[2]) << "";
        return tmp.str();
     }

public:
    // point id
    unsigned int _id;
    // array with 3D coordinates
    double* _xyz;
    // range value
    float _range;
    // array with polar coordinates
    float* _tp;
    // intensity value
    float _intensity;
	// 3 channel color (red, green, blue)
	unsigned char color[3];

};

