#include "LaserPoint.h"

// code from Scan2Image

// C'tor
LaserPoint::LaserPoint()
: _intensity(0.0f)
, _range(0.0f)
, _id(0){
    _xyz = 0;
    _tp = 0;
}
// C'tor

LaserPoint::LaserPoint(int v)
: _intensity(0.0f)
, _range(0.0f)
, _id(0){
    // values are initalized as necessary, indicated by v
    _tp = 0;
    _xyz = 0;

    if(v == 2){
        _tp = new float[2];
        _tp[0] = 0.0f;
        _tp[1] = 0.0f;
    }else if(v == 3){
        _xyz = new double[3];
        _xyz[0] = 0.0f;
        _xyz[1] = 0.0f;
        _xyz[2] = 0.0f;
    } else {
        _tp = new float[2];
        _tp[0] = 0.0f;
        _tp[1] = 0.0f;
        _xyz = new double[3];
        _xyz[0] = 0.0f;
        _xyz[1] = 0.0f;
        _xyz[2] = 0.0f;
    }
}

// copy C'tor
LaserPoint::LaserPoint(const LaserPoint& orig)
: _intensity(orig._intensity)
, _range(orig._range)
, _id(orig._id){
    if(orig._tp != 0){
        _tp = new float[2];
        _tp[0] = orig._tp[0];
        _tp[1] = orig._tp[1];
    }
    if(orig._xyz != 0){
        _xyz = new double[3];
        _xyz[0] = orig._xyz[0];
        _xyz[1] = orig._xyz[1];
        _xyz[2] = orig._xyz[2];
    }
	color[0] = orig.color[0];
	color[1] = orig.color[1];
	color[2] = orig.color[2];
}

// D'tor
LaserPoint::~LaserPoint() {
    // tidy up
    if(_tp != 0)
        delete[] _tp;
    _tp = 0;
    if(_xyz != 0)
        delete[] _xyz;
    _xyz = 0;
}

 