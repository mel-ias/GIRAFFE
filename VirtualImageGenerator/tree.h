//Frank Liebold
//05.01.2015


#ifndef TREE_H
#define TREE_H

#include "nachbardaten.h"
#include "vek.h"
#include "aabb.h"
#include <utility>
////#include "knoten.h"

template <class T, size_t dim>
class Tree{
    
protected:
    //pointer on point list
    std::vector< std::pair<unsigned int, const Vek<T, dim>* > > _pPointList;
    //maxDepth
    unsigned int _maxTiefe;
    //number of Points to finish tree to leafnode
    unsigned int _anzahlPunkteAbbruch;
    
public:
    //Standardkonstruktor
    Tree():_maxTiefe(10), _anzahlPunkteAbbruch(50){}
    
    //2.Konstruktor
    Tree(
        const std::vector<Vek<T, dim> >& pointList,
        unsigned int maxDepth = 10,
        unsigned int numPointsAbort = 50
    ) : _maxTiefe(maxDepth), _anzahlPunkteAbbruch(numPointsAbort) {
        unsigned int i = 0;
        for(auto& it : pointList)
            _pPointList.push_back(std::make_pair(i++, &it));
    }
    
    Tree(
        unsigned int maxDepth ,
        unsigned int numPointsAbort 
    ) : _maxTiefe(maxDepth), _anzahlPunkteAbbruch(numPointsAbort){}
    
    virtual ~Tree(){}
    
    //################################################
    //Klassenfunktionen
    
    ////////////////////////////////////
    void insertPointList(const std::vector<Vek<T, dim> >& pointList){
        _pPointList.clear();
        unsigned int i = 0;
        for(auto& it : pointList)
            _pPointList.push_back(std::make_pair(i++, &it));
    }
    
    void insertPoint(const Vek<T, dim>& point) {
        _pPointList.push_back(std::make_pair(_pPointList.size(), &point));
    }
     
    void insertPoint(unsigned int idx, const Vek<T, dim>& point) {
        _pPointList.push_back(std::make_pair(idx, &point));
    }
    
    //umbenennen in initialize()
    virtual void init() = 0;
    
    //umbenennen in closestPoint()
    virtual NachbarDaten<T> nn(const Vek<T, dim>& anfragePunkt) const = 0;
    
    ////////////////////////////////////+
    //umbenennen in kNearestNeighbors()
    virtual void knn(
        std::vector<NachbarDaten<T> >& kNachbarn,
        const Vek<T, dim>& anfragePunkt,
        size_t k
    ) const = 0;
    
    //neighborsWithinRadius()
    virtual void nachbarnImUmkreis(
        std::vector<NachbarDaten<T> > &nachbarDaten,
        const Vek<T, dim>& suchPunkt,
        const T& radius
    ) const = 0;
    
    virtual void insertAABB(
        const AABB<T, dim> &aabb,
        unsigned int idx//Index der BBox  
    ) = 0;
    
    virtual void nachbarnInAABB(
        std::vector<unsigned int> &nachbarIndizes,
        const AABB<T, dim>& aabb
    ) const = 0;
    
    //neighborsInAABB()
    virtual void nachbarnInAABB(
        std::vector<NachbarDaten<T> > &nachbarDaten,
        const Vek<T, dim>& suchPunkt,
        const T& radius
    ) const = 0;
    
    //!!!!!!!!
    //nodesIndicesLists
    virtual void durchlaufeneKnotenIndizesListen(
        const Vek<T, dim> &start,
        const Vek<T, dim> &richtungsVek,
        std::vector< const std::vector<unsigned int>* >& indizesListenZeiger
    ) const = 0;
    
    //setMaxDepth
    //Tiefe, bei der Knoten auf Blattknoten gesetzt wird
    void setMaxTiefe(unsigned int maxTiefe){
        _maxTiefe = maxTiefe;
    }
    
    //Anzahl der Punkte bei der Knoten auf Blattknoten gesetzt wird
    void setAnzahlPunkteAbbruch(unsigned int anzahl){
        _anzahlPunkteAbbruch = anzahl;
    }
        
    virtual bool istInitialisiert() const = 0;
    
    virtual void clear() = 0;
};


#endif //TREE_H