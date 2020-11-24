/* 
 * File:   aabb.h
 * Author: Frank Liebold
 *
 * Created on 2. Januar 2013, 16:08
 */

#ifndef AABB_H
#define	AABB_H

#include <cmath>

#include "vek.h"

#include <ostream>
#include <algorithm>
#include <limits>

//Struktur, die Koordinaten eines achsparallelen Quaders vereint
//axis aligned Boundingbox 3D
//Huellvolumenquader
//#ifndef ACHSPARALQUADER3D_H
//#define ACHSPARALQUADER3D_H
template <class T, size_t dim>
class AABB{
    
private:
    
public:
    
    Vek<T, dim> pmin;
    Vek<T, dim> pmax;
    
    //Standardkonstruktor
    AABB(){}
    
    //3D-Fall
    AABB(const T& xmin, const T& ymin, const T& zmin,
        const T& xmax, const T& ymax, const T& zmax):
        pmin(xmin, ymin, zmin), pmax(xmax, ymax, zmax){}
    
    //2D-Fall
    AABB(const T& xmin, const T& ymin,
        const T& xmax, const T& ymax) : pmin(xmin, ymin), pmax(xmax, ymax){}
    
    //Klassenfunktionen
    inline T& xmin(){return pmin[0];}
    inline T& xmax(){return pmax[0];}
    inline T& ymin(){return pmin[1];}
    inline T& ymax(){return pmax[1];}
    inline T& zmin(){return pmin[2];}
    inline T& zmax(){return pmax[2];}
    
    inline const T& xmin() const {return pmin[0];}
    inline const T& xmax() const {return pmax[0];}
    inline const T& ymin() const {return pmin[1];}
    inline const T& ymax() const {return pmax[1];}
    inline const T& zmin() const {return pmin[2];}
    inline const T& zmax() const {return pmax[2];}
    
    inline const T& min(size_t i) const{
        assert(i < dim);
        return pmin[i];
    }
    
    inline T& min(size_t i) {
        assert(i < dim);
        return pmin[i];
    }
    
    inline const T& max(size_t i) const{
        assert(i < dim);
        return pmax[i];
    }
    
    inline T& max(size_t i) {
        assert(i < dim);
        return pmax[i];
    }
    
    friend std::ostream& operator<<(std::ostream &os, const AABB<T,dim>& aabb){
        return os << "pmin = " << aabb.pmin << "  pmax = " << aabb.pmax;
    }

    //vorzeichenbehafteter Abstand zu Boundingbox (positiv, wenn aussen)
    inline T abstandVorz(const Vek3<T>& punkt) const{
        
//        int aussen[3];
//        T dist[3];
        T hilfmin, hilfmax;
        T erg(0), erg2,
        erg3 = std::numeric_limits<T>::lowest();
        int sum = 0;
//        for(size_t i = 0; i < 3; ++i){
//            //x-Richtung i=0
//            if( (hilfmin=aabbmin(i)-punkt[i]) > 0 ){
//                //aussen[i]=1;
//                //dist[i]=hilfmin;
//                erg2=hilfmin;
//                erg+=hilfmin*hilfmin;
//                ++sum;
//            }
//            else if( (hilfmax=punkt[i]-aabbmax(i)) > 0 ){
//                //aussen[i]=1;
//                //dist[i]=hilfmax;
//                erg2=hilfmax;
//                erg+=hilfmax*hilfmax;
//                ++sum;
//            }
//            else{
////                aussen[i]=0;
////                dist[i]=std::max(hilfmin,hilfmax);
//                //Wert ist negativ,deshalb max
//                erg3=std::max(erg3,std::max(hilfmin,hilfmax));
//            }
////            sum+=aussen[i];
//        }
        
        //x-Richtung
//        if( (hilfmin=pmin.x()-punkt.x()) > 0 ){
//            erg2=hilfmin;
//            erg+=hilfmin*hilfmin;
//            ++sum;
//        }
//        else if( (hilfmax=punkt.x()-pmax.x()) > 0 ){
//            erg2=hilfmax;
//            erg+=hilfmax*hilfmax;
//            ++sum;
//        }
//        else{
//            //Wert ist negativ,deshalb max
//            erg3=std::max(erg3,std::max(hilfmin,hilfmax));
//        }
//        //y-Richtung
//        if( (hilfmin=pmin.y()-punkt.y()) > 0 ){
//            erg2=hilfmin;
//            erg+=hilfmin*hilfmin;
//            ++sum;
//        }
//        else if( (hilfmax=punkt.y()-pmax.y()) > 0 ){
//            erg2=hilfmax;
//            erg+=hilfmax*hilfmax;
//            ++sum;
//        }
//        else{
//            //Wert ist negativ,deshalb max
//            erg3=std::max(erg3,std::max(hilfmin,hilfmax));
//        }
//        //z-Richtung
//        if( (hilfmin=pmin.z()-punkt.z()) > 0 ){
//            erg2=hilfmin;
//            erg+=hilfmin*hilfmin;
//            ++sum;
//        }
//        else if( (hilfmax=punkt.z()-pmax.z()) > 0 ){
//            erg2=hilfmax;
//            erg+=hilfmax*hilfmax;
//            ++sum;
//        }
//        else{
//            //Wert ist negativ,deshalb max
//            erg3=std::max(erg3,std::max(hilfmin,hilfmax));
//        }
        
        for(unsigned int i = 0; i < dim; ++i)
            if( (hilfmin = pmin[i] - punkt[i]) > 0 ){
                erg2 = hilfmin;
                erg += hilfmin*hilfmin;
                ++sum;
            }
            else if( (hilfmax = punkt[i] - pmin[i]) > 0 ){
                erg2 = hilfmax;
                erg += hilfmax*hilfmax;
                ++sum;
            }
            else{
                //Wert ist negativ, deshalb max
                //aktualisiere ggf. erg3
                erg3 = std::max(erg3, std::max(hilfmin, hilfmax));
            }
        
        if(sum == 0) return erg3;//Punkt liegt innen
        //std::max(dist[0],std::max(dist[1],dist[2]));
        else if(sum == 1) return erg2;//Punkt liegt bezueglich einer Koordinate aussen
//        if(aussen[0]==0 || aussen[1]==0 || aussen[2]==0){
//            return std::max(dist[0],std::max(dist[1],dist[2]));
//        }
        return sqrt(erg);//Pythagoras
    }
    
    
    
    //quadrierter Abstand zu Boundingbox
    inline T quadrierterAbstand(const Vek<T, dim>& punkt) const{
        
//        int aussen[3];
        // T dist[3];
        T hilf;
        T erg(0);//erg2,
        
        //x-Richtung
//        if( (hilf = pmin.x() - punkt.x()) > 0 ) erg += hilf*hilf;
//        else if( (hilf = punkt.x() - pmax.x()) > 0 ) erg += hilf*hilf;
//        //y-Richtung
//        if( (hilf = pmin.y() - punkt.y()) > 0 ) erg += hilf*hilf;
//        else if( (hilf = punkt.y() - pmax.y()) > 0 ) erg += hilf*hilf;
//        //z-Richtung
//        if( (hilf = pmin.z() - punkt.z()) > 0 ) erg += hilf*hilf;
//        else if( (hilf = punkt.z() - pmax.z()) > 0 ) erg += hilf*hilf;
        
        for(size_t i = 0; i < dim; ++i)
            if( (hilf = pmin[i] - punkt[i]) > 0 ) erg += hilf*hilf;
            else if( (hilf = punkt[i] - pmax[i]) > 0 ) erg += hilf*hilf;
        
        return erg;//Pythagoras
    }
    
    //Abstand des Punktes zur AABB
    inline T abstand(const Vek<T, dim>& punkt) const{
        T hilfmin, hilfmax;
        T erg(0), erg2;
        int sum = 0;
////        //x-Richtung
////        if( (hilfmin = pmin.x()-punkt.x()) > 0 ){
////            erg2 = hilfmin;
////            erg += hilfmin*hilfmin;
////            ++sum;
////        }
////        else if( (hilfmax = punkt.x() - pmax.x()) > 0 ){
////            erg2 = hilfmax;
////            erg += hilfmax*hilfmax;
////            ++sum;
////        }
////        //y-Richtung
////        if( (hilfmin = pmin.y() - punkt.y()) > 0 ){
////            erg2 = hilfmin;
////            erg += hilfmin*hilfmin;
////            ++sum;
////        }
////        else if( (hilfmax = punkt.y() - pmax.y()) > 0 ){
////            erg2 = hilfmax;
////            erg += hilfmax*hilfmax;
////            ++sum;
////        }
////        //z-Richtung
////        if( (hilfmin = pmin.z() - punkt.z()) > 0 ){
////            erg2 = hilfmin;
////            erg += hilfmin*hilfmin;
////            ++sum;
////        }
////        else if( (hilfmax = punkt.z() - pmax.z()) > 0 ){
////            erg2 = hilfmax;
////            erg += hilfmax*hilfmax;
////            ++sum;
////        }
        
        for(size_t i = 0; i < dim; ++i)
            if( (hilfmin = pmin[i] - punkt[i]) > 0 ){
                erg2 = hilfmin;
                erg += hilfmin*hilfmin;
                ++sum;
            }
            else if( (hilfmax = punkt[i] - pmax[i]) > 0 ){
                erg2 = hilfmax;
                erg += hilfmax*hilfmax;
                ++sum;
            }
        
        if(sum == 0) return 0;//std::max(dist[0],std::max(dist[1],dist[2]));
        else if(sum == 1) return erg2;
        return sqrt(erg);
    }
   
    //#######################################################
    //Schnitttest Strahl achsparalleler Quader AABB
    //Funktion
    //bestimmt,ob ein Strahl mit Quellpunkt start und Richtungsvektor rvek
    //den achsenparallelen Quader eines Knotens schneidet
    //t0,t1 sind die Parameter des Strahls start+t*rvek fuer Ein- und Austrittspunkt
    inline bool strahlSchneidetAABB(
            const Vek3<T> &ursprung,
            const Vek3<T> &richtung,
            T &t0,
            T &t1
    ) const{
       
        //Hilfsvariable
        T invrveki;
        //Schnitt mit x-Ebene xmin bzw. xmax
        T t0x(0), t1x(0);
        bool sonderfallx = false;
        if(richtung.x() > std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.x();
            t0x = (pmin.x() - ursprung.x())*invrveki;
            t1x = (pmax.x() - ursprung.x())*invrveki;
        }
        else if(richtung.x() < -std::numeric_limits<T>::epsilon() ){//0){
            invrveki=static_cast<T>(1)/richtung.x();
            t0x = (pmax.x() - ursprung.x())*invrveki;
            t1x = (pmin.x() - ursprung.x())*invrveki;
        }
        else{
            if(ursprung.x() < pmin.x() || ursprung.x() > pmax.x()) return false;
            else sonderfallx = true;
        }
        
        //Schnitt mit y-Ebene ymin bzw. ymax
        T t0y(0), t1y(0);
        bool sonderfally = false;
        
        if(richtung.y() > std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.y();
            t0y = (pmin.y() - ursprung.y())*invrveki;
            t1y = (pmax.y() - ursprung.y())*invrveki;
        }
        else if(richtung.y() < -std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.y();
            t0y = (pmax.y() - ursprung.y())*invrveki;
            t1y = (pmin.y() - ursprung.y())*invrveki;
        }
        else{
           if(ursprung.y() < pmin.y() || ursprung.y() > pmax.y()) return 0;
           else sonderfally = true;
        }
        
        //Schnittmenge tx ty
        T maxt0xy(0), mint1xy(0);
        
        if(!sonderfallx && !sonderfally){
            maxt0xy = (t0x > t0y) ? t0x : t0y;
            mint1xy = (t1x < t1y) ? t1x : t1y;
            //falls die Schnittmenge = leere Menge return 0
            if(maxt0xy > mint1xy) return 0;
        }
        else if(sonderfallx && !sonderfally){
            maxt0xy = t0y;
            mint1xy = t1y;
        }
        else if(!sonderfallx && sonderfally){
            maxt0xy = t0x;
            mint1xy = t1x;
        }
        
        //Schnitt mit z-Ebene zmin bzw. zmax
        T t0z(0), t1z(0);
        bool sonderfallz = false;
        
        if(richtung.z() > std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.z();
            t0z = (pmin.z() - ursprung.z())*invrveki;
            t1z = (pmax.z() - ursprung.z())*invrveki;
        }
        else if(richtung.z() < -std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.z();
            t0z = (pmax.z() - ursprung.z())*invrveki;
            t1z = (pmin.z() - ursprung.z())*invrveki;
        }
        else{
            if(ursprung.z() < pmin.z() || ursprung.z() > pmax.z()) return 0;
            else sonderfallz = true;
        }
        
        //Schnittmenge mit der oberen Schnittmenge mit tz0,tz1 -> gesamte Schnittmenge
        T maxt0xyz(0), mint1xyz(0);
        if(sonderfallx && sonderfally){
            maxt0xyz = t0z;
            mint1xyz = t1z;
        }
        else{
            if(!sonderfallz){
                maxt0xyz = (t0z > maxt0xy) ? t0z : maxt0xy;
                mint1xyz = (t1z < mint1xy) ? t1z : mint1xy;
                //falls die Schnittmenge = leere Menge return 0
                if(maxt0xyz > mint1xyz) return false;
            }
            else{
                maxt0xyz = maxt0xy;
                mint1xyz = mint1xy;
            }
        }
        
        //Testen, ob der Testparameter < 0
        //Strahl nur in eine Richtung !!!
        if(mint1xyz < 0 ) return false;
        //Nullsetzen von t0 wenn < 0
        if(maxt0xyz < 0 ) maxt0xyz = 0;
        
        t0 = maxt0xyz;
        t1 = mint1xyz;
        
        return true;
    }
    
    
    //#######################################################
    //Schnitttest Strahl achsparalleles Rechteck AABB
    //Funktion
    //bestimmt,ob ein Strahl mit Quellpunkt start und Richtungsvektor rvek
    //das achsenparallele Rechteck eines Knotens schneidet
    //t0,t1 sind die Parameter des Strahls start+t*rvek fuer Ein- und Austrittspunkt
    inline bool strahlSchneidetAABB(
            const Vek2<T> &ursprung,
            const Vek2<T> &richtung,
            T &t0,
            T &t1
    ) const {
       
        //Hilfsvariable
        T invrveki;
        //Schnitt mit x-Achse xmin bzw. xmax
		//(xmin,y) = ursprung + t*rvek
		//t = (xmin - ursprung.x)/rvek.x
        T t0x(0), t1x(0);
        bool sonderfallx = false;
        if(richtung.x() > std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/ richtung.x();
            t0x = (pmin.x() - ursprung.x())*invrveki;
            t1x = (pmax.x() - ursprung.x())*invrveki;
        }
        else if(richtung.x() < -std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/ richtung.x();
            t0x = (pmax.x() - ursprung.x())*invrveki;
            t1x = (pmin.x() - ursprung.x())*invrveki;
        }
        else{//Strahlrichtung parallel y-Achse -> Probe, ob im Rechteck
            if(ursprung.x() < pmin.x() || ursprung.x() > pmax.x()) return 0;
            else sonderfallx = true;
        }
        
        
        //Schnitt mit y-Achse ymin bzw. ymax
        T t0y(0), t1y(0);
        bool sonderfally = false;
        //Schnitt mit x-Achse xmin bzw. xmax
            //(x,ymin) = start + t*rvek
            //t = (ymin - start.y)/rvek.y
        
        if(richtung.y() > std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.y();
            t0y = (pmin.y() - ursprung.y())*invrveki;
            t1y = (pmax.y() - ursprung.y())*invrveki;
        }
        else if(richtung.y() < -std::numeric_limits<T>::epsilon() ){//0){
            invrveki = static_cast<T>(1)/richtung.y();
            t0y = (pmax.y() - ursprung.y())*invrveki;
            t1y = (pmin.y() - ursprung.y())*invrveki;
        }
        else{//Strahlrichtung parallel x-Achse
           if(ursprung.y() < pmin.y() || ursprung.y() > pmax.y()) return 0;
           else sonderfally = true;
        }
        
        //Schnittmenge tx ty
        T maxt0xy(0), mint1xy(0);
        
        if(!sonderfallx && !sonderfally){
            maxt0xy = (t0x > t0y) ? t0x : t0y;
            mint1xy = (t1x < t1y) ? t1x : t1y;
            //falls die Schnittmenge = leere Menge return 0
            if(maxt0xy > mint1xy) return 0;
        }
        else if(sonderfallx && !sonderfally){
            maxt0xy = t0y;
            mint1xy = t1y;
        }
        else if(!sonderfallx && sonderfally){
            maxt0xy = t0x;
            mint1xy = t1x;
        }
   
        //Testen, ob der Testparameter < 0
        //Strahl nur in eine Richtung !!!
        if(mint1xy < 0 ) return 0;
        //Nullsetzen von t0 wenn < 0
        if(maxt0xy < 0 ) maxt0xy = 0;
        
        t0 = maxt0xy;
        t1 = mint1xy;
        
        return 1;
    }
    
    
    //testet, ob Punkt in AABB liegt
    inline bool punktInAABB(const Vek<T, dim>& punkt) const {
        for(size_t i = 0; i < dim; ++i)
            if(punkt[i] < pmin[i] || punkt[i] > pmax[i]) return false;
        return true;
//        return punkt.x() >= pmin.x() && punkt.x() <= pmax.x() &&
//               punkt.y() >= pmin.y() && punkt.y() <= pmax.y() &&
//               punkt.z() >= pmin.z() && punkt.z() <= pmax.z();
    }
    
    //##################################################
    //testet, ob Klassen-AABB in der uebergebenen AABB liegt
    //ist das der Fall wird 1 zurueckgegeben
    inline bool inAABB( const AABB<T,dim> &aabb) const {
        for(size_t i = 0; i < dim; ++i)
            if(pmin[i] < aabb.pmin[i] || pmax[i] > aabb.pmax[i]) return false;
        return true;
//        return  pmin.x() >= aabb.xmin() && pmin.y() >= aabb.ymin() &&
//                pmin.z() >= aabb.zmin() && pmax.x() <= aabb.xmax() &&
//                pmax.y() <= aabb.ymax() && pmax.z() <= aabb.zmax();
    }
    
    //##############################
    //testet, ob Klassen-AABB1 die uebergebene AABB schneidet
    //Testet, ob der Suchpunkt und der achsenparallele Wuerfel (Eckpunkte in den Koordinaten um radius verschoben)
    //teilweise im achsparallelen Quader des zugehoerigen Knotens liegt
    //ist das der Fall wird 1 zurueckgegeben
//    inline typename std::enable_if<std::is_floating_point<T>::value, bool>::type
//    schneidetAABB(const AABB<T,dim> &aabb) const{
//        for(size_t i=0; i<dim; ++i)
//            if(std::fmax(pmin[i],aabb.pmin[i]) > std::fmin(pmax[i],aabb.pmax[i]))
//                return false;
//        return true;
//    }
//    inline typename std::enable_if<std::is_integral<T>::value, bool>::type
//    schneidetAABB(const AABB<T,dim> &aabb) const{
//        for(size_t i=0; i<dim; ++i)
//            if(std::max(pmin[i],aabb.pmin[i]) > std::min(pmax[i],aabb.pmax[i]))
//                return false;
//        return true;
//    }
    inline bool schneidetAABB(const AABB<T, dim> &aabb) const {
        for(size_t i = 0; i < dim; ++i)
            if(std::max(pmin[i], aabb.pmin[i]) > std::min(pmax[i], aabb.pmax[i]))
                return false;
        return true;
//        return  std::max(pmin.x(),aabb.xmin()) <= std::min(pmax.x(),aabb.xmax()) &&
//                std::max(pmin.y(),aabb.ymin()) <= std::min(pmax.y(),aabb.ymax()) &&
//                std::max(pmin.z(),aabb.zmin()) <= std::min(pmax.z(),aabb.zmax());
    }
    
//    inline bool schneidetAABB(const AABB<float,dim> &aabb) const{
//        bool erfolg=1;
//        for(size_t i=0;i<dim;++i)
//            if(std::fmax(pmin[i],aabb.pmin[i]) > std::fmin(pmax[i],aabb.pmax[i])){
//                erfolg = 0 ;
//                break;
//            }
//        return erfolg;
//    }
//    
//    inline bool schneidetAABB(const AABB<double,dim> &aabb) const{
//        bool erfolg=1;
//        for(size_t i=0;i<dim;++i)
//            if(std::fmax(pmin[i],aabb.pmin[i]) > std::fmin(pmax[i],aabb.pmax[i])){
//                erfolg = 0 ;
//                break;
//            }
//        return erfolg;
//    }
    
    //##############################
    //testet ob Kugel AABB-Quader schneidet
    //Testet, ob die Kugel um den Suchpunkt herum den 
    //achsparallelen Quader des zugehoerigen Knotens schneidet
    //ist das der Fall wird 1 zurueckgegeben
//    inline bool kugelSchneidetAABB(const Vek3<T> &mittelpunkt,
//                                   const T &radiusQuadrat) const{
//        T distQu(0);
//        T hilf;
//        
//        if((hilf = pmin.x() - mittelpunkt.x()) > 0) distQu += hilf*hilf;
//        else if((hilf = mittelpunkt.x() - pmax.x()) > 0) distQu += hilf*hilf;
//        
//        if((hilf = pmin.y() - mittelpunkt.y()) > 0) distQu += hilf*hilf;
//        else if((hilf = mittelpunkt.y() - pmax.y()) > 0) distQu += hilf*hilf;
//        
//        if((hilf = pmin.z() - mittelpunkt.z()) > 0) distQu += hilf*hilf;
//        else if((hilf = mittelpunkt.z() - pmax.z()) > 0) distQu += hilf*hilf;
//        
//        return distQu <= radiusQuadrat;
//    }
    
    static void berechneAABB(
            AABB<T, dim>& aabb,
            const std::vector<Vek<T, dim> >& punkte
    ){

        auto it = punkte.cbegin();
        auto itend = punkte.cend();
        size_t i;
        if(it != itend){
            for(i = 0; i < dim; ++i)
                aabb.pmin[i] = aabb.pmax[i] = it->operator[](i);
            ++it;
            if(it != itend)
                for(; it != itend; ++it) {
                    for(i = 0; i < dim; ++i)
                        if(it->operator[](i) > aabb.pmax[i])
                            aabb.pmax[i] = it->operator[](i);
                        else if(it->operator[](i) < aabb.pmin[i])
                            aabb.pmin[i] = it->operator[](i);
                }
        }
    }

};
//#endif


template<class T>
using AABB3 = AABB<T, 3>;

template<class T>
using AABB2 = AABB<T, 2>;


typedef AABB<double, 3> AABB3d;
typedef AABB<float, 3> AABB3f;
typedef AABB<double, 2> AABB2d;
typedef AABB<float, 2> AABB2f;

               
#endif	/* AABB_H */

