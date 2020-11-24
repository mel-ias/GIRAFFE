//Frank Liebold
//09.05.2014


#ifndef KDTREEKNOTEN_H
#define KDTREEKNOTEN_H

#include <vector>
#include <cassert>
#include <iostream>
#include <cstddef>
using std::cout;
using std::endl;

/////////////////////////////
//KdTree-Knoten-Klasse
//abstrakt
template <class T>
class KdTreeKnoten {

public:

    //KdTreeKnoten() : _istBlatt(true) {}
    KdTreeKnoten(){}

    //destructor
    virtual ~KdTreeKnoten() { }

    //functions
    virtual bool istBlatt() const = 0;
    
};

/////////////////////////////////////////////////////
//abgeleitete Klasse fuer Elternknoten
template< class T >
class KdTreeElternKnoten : public KdTreeKnoten<T>
{
public:
    KdTreeKnoten<T> *links;
    KdTreeKnoten<T> *rechts;
    T separator;
    unsigned char dimSeperation;

    KdTreeElternKnoten() : links(nullptr), rechts(nullptr),
            separator(0), dimSeperation(0){}
    
    ~KdTreeElternKnoten(){
        if(links != nullptr) delete links;
        if(rechts != nullptr) delete rechts;
    }
    
    ///////////////////////////////////////////////
    //Klassenfunktionen
    bool istBlatt() const{
        return false;
    }
    
};

/////////////////////////////////////////////////////
//abgeleitete Klasse fuer Blattknoten
template< class T >
class KdTreeBlattKnoten : public KdTreeKnoten<T>
{
public:
    unsigned int idxBegin;
    unsigned int idxEnd;
    std::vector<unsigned int> liste;//for triangles,AABBs ...

    KdTreeBlattKnoten(unsigned int idxBegin, unsigned int idxEnd)
    : idxBegin(idxBegin), idxEnd(idxEnd){}
    
    ~KdTreeBlattKnoten(){}
  
    ///////////////////////////////////////////////
    //Klassenfunktionen
    bool istBlatt() const{
        return true;
    }
};


//////////////////////////////////////
//Klasse fuer Strahlschnitt mit Knoten-AABB
template <class T>
class DurchlaufenerKdTreeKnoten{
public:
    const KdTreeKnoten<T>* knoten; //Knotenzeiger
    T t0;  //Geradenparameter des Eintrittschnittpunktes bzw. 0
    T t1;  //Geradenparameter des Austrittschnittpunktes
    
    //Konstruktor
    DurchlaufenerKdTreeKnoten(){}
    DurchlaufenerKdTreeKnoten(
        const KdTreeKnoten<T>* knoten,
        const T& t0,
        const T& t1) : knoten(knoten), t0(t0), t1(t1){}
    
    inline bool operator<(const DurchlaufenerKdTreeKnoten<T>& k) const {
        return t1 < k.t1;
    }

};


typedef KdTreeKnoten<double> KdTreeKnotend;
typedef KdTreeKnoten<float> KdTreeKnotenf;

typedef KdTreeElternKnoten<double> KdTreeElternKnotend;
typedef KdTreeElternKnoten<float> KdTreeElternKnotenf;

typedef KdTreeBlattKnoten<double> KdTreeBlattKnotend;
typedef KdTreeBlattKnoten<float> KdTreeBlattKnotenf;

typedef DurchlaufenerKdTreeKnoten<double> DurchlaufenerKdTreeKnotend;
typedef DurchlaufenerKdTreeKnoten<float> DurchlaufenerKdTreeKnotenf;

#endif //KDTREEKNOTEN_H

