//Frank Liebold
//17.04.2014

#ifndef VEK_H
#define VEK_H

#include <algorithm>
#include <ostream>
#include <cmath>
#include <numeric>
#include <array>
#include <cassert>

//Quaternionenklasse standardtype T  ->  class T
//bestimmte funktionen fuer T reservieren

//########################################################
//nD
template <class T, size_t dim>
class Vek
{
private:
    //Membervariablen
    std::array<T, dim> _data;
    //T _data[dim];
    
public:

    //Konstruktor
    inline Vek(){
        std::fill(_data.begin(), _data.end(), static_cast<T>(0));
    }
    
    inline Vek(const T& x, const T& y, const T& z) : _data({x, y, z}){
//        _data[0] = x; _data[1] = y; _data[2] = z;
    }
    
    inline Vek(const T& x, const T& y):_data({x, y}){
//        _data[0] = x; _data[1] = y;
    }
    
    inline Vek(const T& x, const T& y, const T& z, const T& w)
        : _data({x, y, z, w}){}

    //Destruktor
    //~Vek(){}

    inline const T* data() const {return _data.data();}
    
    inline T* data() { return _data.data();}
    
    ////////////////////////////////////////////
    //Operatoren
    
    //Vergleichsoperator
    inline bool operator==(const Vek<T, dim>& v){
        return std::equal(_data.data(), _data.data() + dim, v.data());
    }
    
    inline bool operator!=(const Vek<T, dim>& v){
        return !std::equal(_data.data(), _data.data() + dim, v.data());
    }

    // Klammeroperator
    inline const T& operator[](size_t i) const {
//        assert(i < dim);
        return _data[i];
//        return *(_data.data() + i);
    }

    inline T& operator[](size_t i) {
//        assert(i < dim);
        return _data[i];
//        return *(_data.data() + i);
    }

    inline const T& operator()(size_t i) const {
        assert(i < dim);
        return _data[i];
    }

    inline T& operator()(size_t i) {
        assert(i < dim);
        return _data[i];
    }

    /** Subtraktionsoperator */
    inline const Vek<T, dim>  operator-(const Vek<T, dim>& v) const {
        Vek<T, dim> diff;
//        for(size_t i = 0; i < dim; ++i) diff[i] = _data[i] - v[i];
        auto zDiff = diff.data();
        auto zV = v.data();
        for(auto& d:_data) *zDiff++ = d - *zV++;
        
        return diff;
    }

    inline Vek<T, dim>& operator-=(const Vek<T, dim>& v) {
//        for(size_t i = 0; i < dim; ++i) _data[i] -= v[i];
        auto zV = v.data();
        for(auto& d : _data) d -= *zV++;
        return *this;
    }

    // Additionsoperator
    inline const Vek<T, dim>  operator+(const Vek<T, dim>& v) const {
        Vek<T, dim> sum;
//        for(size_t i = 0; i < dim; ++i) sum[i] = _data[i] + v[i];
        auto zSum = sum.data();
        auto zV = v.data();
        for(auto& d:_data) *zSum++ = d + *zV++;
        return sum;
    }

    inline Vek<T, dim>& operator+=(const Vek<T, dim>& v) {
//        for(size_t i = 0; i < dim; ++i) _data[i] += v[i];
        auto zV = v.data();
        for(auto& d:_data) d += *zV++;
        return *this;
    }

    // Multiplikationsoperator mit Skalar
    inline const Vek<T, dim> operator*(const T& skalar) const {
        Vek<T, dim> prod;
        auto zProd = prod.data();
        //for(size_t i = 0; i < dim; ++i) prod[i] = skalar*_data[i];
        for(auto& d : _data) *zProd++ = d*skalar;
        return prod;
    }

    //Multiplikation mit Skalar
    inline Vek<T, dim>& operator*=(const T& skalar) {
//        for(size_t i = 0; i < dim; ++i) _data[i] *= skalar;
        for(auto& d : _data) d *= skalar;
        return *this;
    }

    
    // Additionsoperator fuer Skalar
    inline Vek<T, dim>& operator+=(const T& v) {
        for(auto& d : _data) d += v;
        return *this;
    }
    
    inline const Vek<T, dim>  operator+(const T& v) const {
        Vek<T, dim> sum;
        auto zSum = sum.data();
        for(auto& d : _data) *zSum++ = d + v;
        return sum;
    }

    // Subtraktionsoperator fuer Skalar
    inline Vek<T, dim>& operator-=(const T& v) {
        for(auto& d : _data) d -= v;
        return *this;
    }
    
    inline const Vek<T, dim>  operator-(const T& v) const {
        Vek<T, dim> diff;
        auto zDiff = diff.data();
        for(auto& d : _data) *zDiff++ = d - v;
        return diff;
    }
    

    // Divisionsoperator Division durch Skalar
    inline Vek<T, dim>  operator/(const T& skalar) const {
        return operator*(static_cast<T>(1)/skalar);
    }

    inline Vek<T, dim>& operator/=(const T& skalar) {
        return *this *= static_cast<T>(1)/skalar;
    }

    //Skalarprodukt
    inline const T operator*(const Vek<T, dim>& v) const {
        return std::inner_product(_data.data(), _data.data() + dim, v.data(),
                static_cast<T>(0));
    }


    ////////////////////////////////////////////////
    //Klassenfunktionen

    //Betrag ins Quadrat
    inline T quadrierterBetrag() const {
        return std::inner_product(_data.begin(), _data.end(), _data.begin(),
                static_cast<T>(0));
    }

    //Betrag
    inline T betrag() const {
        return sqrt(std::inner_product(_data.begin(), _data.end(),
                _data.begin(), static_cast<T>(0)));
    }

    //Normalisierung
    inline bool normieren() {
        T n = std::inner_product(_data.begin(), _data.end(),
                _data.begin(), static_cast<T>(0));
        if(n == static_cast<T>(0)) return 0;//assert(n != static_cast<T>(0));
        n = static_cast<T>(1)/sqrt(n);
        for(auto& it : _data) it *= n;
        return 1;
    }
    
    inline bool normieren(T eps) {
        static const T eins = static_cast<T>(1);
        T n = std::inner_product(_data.begin(), _data.end(), _data.begin(),
                static_cast<T>(0));
        if(n <= eps) return 0;//assert(n != static_cast<T>(0));
        n = eins/sqrt(n);
        for(auto& it : _data) it *= n;
        return 1;
    }

    // Output Stream
    friend std::ostream& operator<<(std::ostream &os, const Vek<T, dim>& v) {
        os << '[';
        auto it = v.data(), itend = v.data() + v.dimension();
        if(it != itend) os << *it;
        ++it;
        for(; it != itend; ++it) os << ", " << *it;
        //for(auto& it : v) os<<it<<", ";
        return os << ']';
    }

    inline const T& x() const {return _data[0];}
    inline T& x() {return _data[0];}

    inline const T& y() const {return _data[1];}
    inline T& y() {return _data[1];}

    inline const T& z() const {return _data[2];}
    inline T& z() {return _data[2];}
    
    inline T distanzZu(const Vek<T, dim>& v) const {
        T hilf2(static_cast<T>(0)), hilf;
        for(size_t i = 0; i < dim; ++i){
            hilf = v[i] - _data[i];
            hilf2 += hilf*hilf;
        }
        return sqrt(hilf2);
    }
    
    inline T quadrierteDistanzZu(const Vek<T,dim>& v) const {
        T hilf2(static_cast<T>(0)), hilf;
        for(size_t i = 0; i < dim; ++i){
            hilf = v[i] - _data[i];
            hilf2 += hilf*hilf;
        }
        return hilf2;
    }
    
    inline size_t dimension() const {return dim;}
    
    inline void multSkalar(const T& skalar) {
        for(auto& it : _data) it *= skalar;
    }
    
    inline void setNull(){
        for(auto& it : _data) it = 0;
    }
    
    inline void setTo(const T& wert){
        for(auto& it : _data) it = wert;
    }
    
    //Kreuzprodukt
    static Vek<T, 3> kreuzProdukt(const Vek<T, 3>& v1, const Vek<T, 3>& v2) {
        Vek<T, 3> v3;
        v3.x() = v1.y()*v2.z() - v1.z()*v2.y();
        v3.y() = v1.z()*v2.x() - v1.x()*v2.z();
        v3.z() = v1.x()*v2.y() - v1.y()*v2.x();
        return v3;
    }

    static void kreuzProdukt(
            Vek<T, 3>& kreuz,
            const Vek<T, 3>& v1,
            const Vek<T, 3>& v2
    ) {
        kreuz.x() = v1.y()*v2.z() - v1.z()*v2.y();
        kreuz.y() = v1.z()*v2.x() - v1.x()*v2.z();
        kreuz.z() = v1.x()*v2.y() - v1.y()*v2.x();
    }

    //Skalarprodukt
    static T skalarProdukt(const Vek<T, dim> &v1, const Vek<T, dim> &v2) {
        return std::inner_product(v1.data(), v1.data() + dim, v2.data(),
                static_cast<T>(0));
    }

    //Skalarprodukt
    static void skalarProdukt(
            T& skalarProd,
            const Vek<T, dim> &v1,
            const Vek<T, dim> &v2
    ) {
        skalarProd = std::inner_product(v1.data(), v1.data() + dim, v2.data(),
                static_cast<T>(0));
    }

    //Winkel zwischen 2 Vektoren in rad
    static T winkel(const Vek<T, dim> &v1, const Vek<T, dim> &v2) {
        T v1_quadBetrag = v1.quadrierterBetrag();
        if(v1_quadBetrag == 0) return 0;
        T v2_quadBetrag = v2.quadrierterBetrag();
        if(v2_quadBetrag == 0) return 0;
        return acos( std::inner_product(v1.data(), v1.data() + dim, v2.data(),
                static_cast<T>(0))/sqrt(v1_quadBetrag*v2_quadBetrag) );
    }

    static void subtrahiereVek(
            Vek<T, dim>& differenz,
            const Vek<T, dim>& v1,
            const Vek<T, dim>& v2
    ) {
        for(size_t i = 0; i < dim; ++i) differenz[i] = v1[i] - v2[i];
    }

    static void addiereVek(
            Vek<T, dim>& sum,
            const Vek<T, dim>& v1,
            const Vek<T, dim>& v2
    ){
        for(size_t i = 0; i < dim; ++i) sum[i] = v1[i] + v2[i];
    }

//    static T quadrierteDistanz(const Vek<T, dim>& v1, const Vek<T, dim>& v2){
//        T hilf2(static_cast<T>(0)), hilf;
//        for(size_t i = 0; i < dim; ++i){
//            hilf = v1[i] - v2[i];
//            hilf2 += hilf*hilf;
//        }
//        return hilf2;
//    }
//
//    static T distanz(const Vek<T, dim>& v1, const Vek<T, dim>& v2){
//        T hilf2(static_cast<T>(0)), hilf;
//        for(size_t i = 0; i < dim; ++i){
//            hilf = v1[i] - v2[i];
//            hilf2 += hilf*hilf;
//        }
//        return sqrt(hilf2);
//    }
    
//    static T distanz(const Vek<T, 2>& v1, const Vek<T, 2>& v2){
//        return hypot(v1[0] - v2[0], v1[1] - v2[1]);
//    }
    
};



//Kreuzprodukt
template <class T>
inline Vek<T, 3> kreuzProdukt(const Vek<T, 3> &v1, const Vek<T, 3> &v2) {
    Vek<T, 3> v3;
    v3.x() = v1.y()*v2.z() - v1.z()*v2.y();
    v3.y() = v1.z()*v2.x() - v1.x()*v2.z();
    v3.z() = v1.x()*v2.y() - v1.y()*v2.x();
    return v3;
}

template <class T>
inline void kreuzProdukt(
        Vek<T, 3>& kreuz,
        const Vek<T, 3>& v1,
        const Vek<T, 3>& v2
){
    kreuz.x() = v1.y()*v2.z() - v1.z()*v2.y();
    kreuz.y() = v1.z()*v2.x() - v1.x()*v2.z();
    kreuz.z() = v1.x()*v2.y() - v1.y()*v2.x();
}

//Skalarprodukt
template <class T, size_t dim>
inline T skalarProdukt(const Vek<T, dim> &v1, const Vek<T, dim> &v2){
    return std::inner_product(v1.data(), v1.data() + dim, v2.data(),
            static_cast<T>(0));
}

//Skalarprodukt
template <class T, size_t dim>
inline void skalarProdukt(
        T& skalarProdukt,
        const Vek<T, dim> &v1,
        const Vek<T, dim> &v2
){
    skalarProdukt = std::inner_product(v1.data(), v1.data() + dim, v2.data(),
            static_cast<T>(0));
}


//Winkel zwischen 2 Vektoren in rad
template <class T, size_t dim>
inline T winkel(const Vek<T, dim> &v1, const Vek<T, dim> &v2) {
    T v1_quadBetrag = v1.quadrierterBetrag();
    if(v1_quadBetrag == 0) return 0;
    T v2_quadBetrag = v2.quadrierterBetrag();
    if(v2_quadBetrag == 0) return 0;
    return acos(std::inner_product(v1.data(), v1.data() + dim, v2.data(),
        static_cast<T>(0))/sqrt(v1_quadBetrag*v2_quadBetrag));
}


template <class T, size_t dim>
inline void subtrahiereVek(
        Vek<T, dim>& diff,
        const Vek<T, dim>& v1,
        const Vek<T, dim>& v2
){
    for(size_t i = 0; i < dim; ++i) diff[i] = v1[i] - v2[i];
}


template <class T, size_t dim>
inline void addiereVek(
        Vek<T, dim>& sum,
        const Vek<T, dim>& v1,
        const Vek<T, dim>& v2
){
    for(size_t i = 0; i < dim; ++i) sum[i] = v1[i] + v2[i];
}


template <class T,size_t dim>
inline T quadrierteDistanz(const Vek<T, dim>& v1, const Vek<T, dim>& v2){
    T hilf2(static_cast<T>(0)), hilf;
    for(size_t i = 0; i < dim; ++i){
        hilf = v1[i] - v2[i];
        hilf2 += hilf*hilf;
    }
    return hilf2;
}


template <class T>
inline T quadrierteDistanz(const Vek<T, 3>& v1, const Vek<T, 3>& v2){
    T hilf2(static_cast<T>(0)), hilf;
    hilf = v1.x() - v2.x();
    hilf2 = hilf*hilf;
    hilf = v1.y() - v2.y();
    hilf2 += hilf*hilf;
    hilf = v1.z() - v2.z();
    hilf2 += hilf*hilf;
    return hilf2;
}


template <class T>
inline T quadrierteDistanz(const Vek<T, 2>& v1, const Vek<T, 2>& v2){
    T hilf2(static_cast<T>(0)), hilf;
    hilf = v1.x() - v2.x();
    hilf2 = hilf*hilf;
    hilf = v1.y() - v2.y();
    hilf2 += hilf*hilf;
    return hilf2;
}


template <class T, size_t dim>
inline T distanz(const Vek<T, dim>& v1, const Vek<T, dim>& v2){
    T hilf2(static_cast<T>(0)), hilf;
    for(size_t i = 0; i < dim; ++i){
        hilf = v1[i] - v2[i];
        hilf2 += hilf*hilf;
    }
    return sqrt(hilf2);
}


template <class T>
inline T distanz(const Vek<T, 3>& v1, const Vek<T, 3>& v2){
    T hilf2, hilf;
    hilf = v1.x() - v2.x();
    hilf2 = hilf*hilf;
    hilf = v1.y() - v2.y();
    hilf2 += hilf*hilf;
    hilf = v1.z() - v2.z();
    hilf2 += hilf*hilf;
    return sqrt(hilf2);
}


template <class T>
inline T distanz(const Vek<T, 2>& v1, const Vek<T, 2>& v2) {
    return hypot(v1.x() - v2.x(), v1.y() - v2.y());
}


template <class T, size_t dim>
inline Vek<T, dim> operator*(const T& skalar, const Vek<T, dim>& v) {
    return v*skalar;
}


template<class T>
using Vek3 = Vek<T, 3>;

template<class T>
using Vek2 = Vek<T, 2>;

template<class T>
using Vek4 = Vek<T, 4>;

typedef Vek<double, 3> Vek3d;
typedef Vek<float, 3> Vek3f;
typedef Vek<int, 3> Vek3i;
typedef Vek<unsigned int, 3> Vek3ui;
typedef Vek<long int, 3> Vek3li;
typedef Vek<unsigned char, 3> Vek3uc;
typedef Vek<bool, 3> Vek3b;
typedef Vek<long double, 3> Vek3ld;
typedef Vek<long unsigned int, 3> Vek3lui;
typedef Vek<short int, 3> Vek3si;

typedef Vek<double, 2> Vek2d;
typedef Vek<float, 2> Vek2f;
typedef Vek<int, 2> Vek2i;
typedef Vek<unsigned int, 2> Vek2ui;
typedef Vek<long int, 2> Vek2li;
typedef Vek<unsigned char, 2> Vek2uc;
typedef Vek<bool, 2> Vek2b;
typedef Vek<long double, 2> Vek2ld;
typedef Vek<long unsigned int, 2> Vek2lui;
typedef Vek<short int, 2> Vek2si;

typedef Vek<double, 4> Vek4d;
typedef Vek<float, 4> Vek4f;
typedef Vek<int, 4> Vek4i;
typedef Vek<unsigned int, 4> Vek4ui;
typedef Vek<long int, 4> Vek4li;
typedef Vek<unsigned char, 4> Vek4uc;
typedef Vek<bool, 4> Vek4b;
typedef Vek<long double, 4> Vek4ld;
typedef Vek<long unsigned int, 4> Vek4lui;
typedef Vek<short int, 4> Vek4si;

#endif