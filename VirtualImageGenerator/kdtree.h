//Frank Liebold
//09.05.14


#ifndef KDTREE_H
#define KDTREE_H

#include <cmath>
#include <cassert>
#include <vector>
#include <stack>//stack to avoid recursion
#include <queue>//priority queue
#include <algorithm> //for sort
#include <iostream>
#include <utility>   // std::pair

#include <numeric> //iota

using std::cout;
using std::endl;

#include "vek.h"
#include "kdtreeknoten.h"
#include "nachbardaten.h"
#include "aabb.h"

#include "tree.h"

template <class T, size_t dim>
class KdTree : public Tree<T, dim>
{

    private:
    //rootnode
    KdTreeKnoten<T>* _wurzel;
    //pointer on point list
//    std::vector<std::pair<unsigned int,const Vek<T,dim>*> > Tree<T,dim>::_pPointList;
//    //maxDepth
//    unsigned int Tree<T,dim>::_maxTiefe;
//    //number of Points to finish tree to leafnode
//    unsigned int Tree<T,dim>::_anzahlPunkteAbbruch;

    //List with Indizes of the points
    
    
    AABB<T, dim> _aabb;

    ///////////////////////////////////
    //structure for neighbor searching
    struct SearchEntry {
    public:
        
        //node
        const KdTreeKnoten<T>* node;
        AABB<T, dim> aabbNode;
        //squared Distance
        //T dist;
        T quadrierteDistanz;
        
//        SearchEntry() : quadrierteDistanz(std::numeric_limits<T>::max()), node(nullptr){}

        SearchEntry(
            const KdTreeKnoten<T>* node,
            const AABB<T,dim>& aabbNode,
            T squaredDistance)
        : node(node), aabbNode(aabbNode), quadrierteDistanz(squaredDistance) {}

        inline bool operator<(const SearchEntry& s) const {
            //> da bei priorityQueue absteigend sortiert werden soll
            return quadrierteDistanz > s.quadrierteDistanz;
        }

    };

    /////////////////////////////////////
    //structur for building the tree
    struct BuildEntry {

        KdTreeKnoten<T>* node;
        AABB<T, dim> aabbNode;
        unsigned int depth;
        unsigned int idxBegin;
        unsigned int idxEnd;
        
        BuildEntry(
            KdTreeKnoten<T>* node,
            const AABB<T,dim>& aabbNode,
            unsigned int depth,
            unsigned int idxBegin,
            unsigned int idxEnd
            ) : node(node), aabbNode(aabbNode), depth(depth),
                idxBegin(idxBegin), idxEnd(idxEnd){}   
    };
    
    //structure to arrange indizes according to coordinates
    struct SortPoints{
        unsigned char dimMax;
        
        SortPoints(unsigned char dimMax):dimMax(dimMax){}
        
        inline bool operator()(
            const std::pair<unsigned int, const Vek<T, dim>*>& i,
            const std::pair<unsigned int, const Vek<T, dim>*>& j
        ) const {
            return i.second->operator[](dimMax) < j.second->operator[](dimMax);
        }
    };

    public:

    //standard constructor
    KdTree() : _wurzel(nullptr) {
        Tree<T,dim>::_maxTiefe = 10;
        Tree<T,dim>::_anzahlPunkteAbbruch = 50;
    }

    //2. constructor
    KdTree( const std::vector<Vek<T,dim> >& pointList,
            unsigned int maxTiefe = 10,
            unsigned int anzPunkteAbbruch = 50
    ) : _wurzel(nullptr) {
        Tree<T,dim>::_maxTiefe = maxTiefe;
        Tree<T,dim>::_anzahlPunkteAbbruch = anzPunkteAbbruch;
        unsigned int i = 0;
        for(auto& it : pointList)
            Tree<T,dim>::_pPointList.push_back(std::make_pair(i++, &it));
    }

    //destructor
    ~KdTree() {
        if(_wurzel != nullptr) delete _wurzel;
    }

    ////////////////////////////////////
    ////////////////////////////////////

    ////////////////////////////////////
//    void insertPointList(const std::vector<Vek<T, dim> >& pointList){
//        Tree<T, dim>::_pPointList.clear();
//        unsigned int i=0;
//        for(auto& it : pointList) Tree<T, dim>::_pPointList.push_back(std::make_pair(i++,&it));
//    }
//    
//    void insertPoint(const Vek<T, dim>& point){
//         Tree<T, dim>::_pPointList.push_back(std::make_pair(Tree<T, dim>::_pPointList.size(),&point));
//    }
//     
//    void insertPoint(unsigned int idx,const Vek<T, dim>& point){
//         Tree<T, dim>::_pPointList.push_back(std::make_pair(idx, &point));
//    }

    inline const AABB<T, dim>& aABB() const{
        return _aabb;
    }
    
    /////////////////////////////////////
    inline void computeAABB(){
        auto it = Tree<T, dim>::_pPointList.cbegin();
        auto itend = Tree<T, dim>::_pPointList.cend();
        size_t i;
        if(it != itend){
            for(i = 0; i < dim; ++i)
                _aabb.pmin[i] = _aabb.pmax[i] = it->second->operator[](i);

            ++it;
            if(it != itend)
                for(; it != itend; ++it)
                    for(i = 0; i < dim; ++i)
                        if(it->second->operator[](i) > _aabb.pmax(i))
                            _aabb.pmax(i) = it->second->operator[](i);
                        else if(it->second->operator[](i) < _aabb.pmin(i))
                            _aabb.pmin(i) = it->second->operator[](i);
        }
    }
    
    ////////////////////////////////////
    void init(){
                
        unsigned int anzahlPunkte = Tree<T, dim>::_pPointList.size();
        if(anzahlPunkte == 0) return;
        
        //Boundingbox
        computeAABB();
        
        KdTreeKnoten<T>* node;
        
        std::stack<BuildEntry> nodeStack;
        
        if(anzahlPunkte < Tree<T, dim>::_anzahlPunkteAbbruch) {//leaf
            _wurzel = new KdTreeBlattKnoten<T>(0, anzahlPunkte);
            return;
        }
        else {//no leaf
            _wurzel = new KdTreeElternKnoten<T>();
        }
                
        nodeStack.push(BuildEntry(_wurzel, _aabb, 0, 0, anzahlPunkte));
        
        unsigned int depth, idxBegin, idxEnd;
        unsigned int idxMitte;
        unsigned int depth_plus_1;
        T median;
        unsigned int i, dimMax;
        Vek<T, dim> deltaAABB;
        T hilf;
        AABB<T, dim> aabbNode;
        
        do{
            BuildEntry& be = nodeStack.top();
            idxBegin = be.idxBegin;
            idxEnd = be.idxEnd;
            node = be.node;
            depth = be.depth;
            aabbNode = be.aabbNode;

            nodeStack.pop();
            
            if(node->istBlatt()) continue;
            
            KdTreeElternKnoten<T>* parent_node = (KdTreeElternKnoten<T>*)node;
            
            //determine the dimension with the largest delta
            for(i = 0; i < dim; ++i)
                deltaAABB[i] = aabbNode.pmax(i) - aabbNode.pmin(i);

            dimMax = 0;
            for(i = 1; i < dim; ++i)
                if(deltaAABB[i] > deltaAABB[dimMax]) dimMax = i;
            
//            if(delx > dely)
//                if(delx > aabbNode.zmax() - aabbNode.zmin()) dimMax = 0;//delx > delz
//                else dimMax = 2;
//            else
//                if(dely > aabbNode.zmax() - aabbNode.zmin()) dimMax = 1;//dely > delz
//                else dimMax = 2;

            depth_plus_1 = depth + 1;
            
            idxMitte = (idxEnd + idxBegin)/2;

            //determine the median element
            //uneven number of elements
            //nth_element ordnet nach mitte-Element, alles davor kleiner,
            //alles danach groesser, sonst ungeordnet
            std::nth_element(Tree<T, dim>::_pPointList.data() + idxBegin,
                Tree<T, dim>::_pPointList.data() + idxMitte,
                Tree<T, dim>::_pPointList.data() + idxEnd, SortPoints(dimMax));
            //median=(_zPunkteVektor->idxBegin()+_idxList[mitte])->x();
            median = Tree<T, dim>::_pPointList[idxMitte].second->operator[](dimMax);
            if(idxEnd - idxBegin % 2 == 0 ){//even number of elements -> average of the middle elements
                //maximal element links of seperator
                auto it = std::max_element(
                            Tree<T, dim>::_pPointList.data() + idxBegin,
                            Tree<T, dim>::_pPointList.data() + idxMitte,
                            SortPoints(dimMax));
                //average
                median = 0.5*( it->second->operator[](dimMax) + median);
            }
            
            parent_node->dimSeperation = dimMax;
            parent_node->separator = median;
            
            //check if leaf, if it is abort
            if(depth_plus_1 == Tree<T, dim>::_maxTiefe ){
                parent_node->links =
                            new KdTreeBlattKnoten<T>(idxBegin, idxMitte);
                parent_node->rechts =
                            new KdTreeBlattKnoten<T>(idxMitte, idxEnd);
                continue;
            }
            if(idxMitte-idxBegin<Tree<T, dim>::_anzahlPunkteAbbruch){//leaf
                parent_node->links =
                                new KdTreeBlattKnoten<T>(idxBegin,idxMitte);
            }
            else{//no leaf
                parent_node->links = new KdTreeElternKnoten<T>();
                //add child node to the stack
                hilf = aabbNode.max(dimMax);
                aabbNode.max(dimMax) = median;
                nodeStack.push(BuildEntry(parent_node->links,aabbNode,
                                        depth_plus_1, idxBegin, idxMitte));
                aabbNode.max(dimMax) = hilf;//restore
            }
            if(idxEnd-idxMitte<Tree<T, dim>::_anzahlPunkteAbbruch){//leaf
                parent_node->rechts =
                                new KdTreeBlattKnoten<T>(idxMitte, idxEnd);
            }
            else{//no leaf
                parent_node->rechts = new KdTreeElternKnoten<T>();
                //add child node to the stack
                aabbNode.min(dimMax) = median;
                nodeStack.push(BuildEntry(parent_node->rechts, aabbNode,
                                            depth_plus_1, idxMitte, idxEnd));
            }
        
        }while(!nodeStack.empty());
        
    }

    //#################################################################
    ////////////////////////////////////
    void considerPath(
        AABB<T, dim>& aabbNode,
        std::priority_queue<SearchEntry>& nodeQueue,
        NachbarDaten<T>& best,
        const KdTreeKnoten<T>* nodeBegin,
        const Vek<T, dim>& anfragePunkt
    ) const{
        
        T quadrierteDistanzLeft,quadrierteDistanzRight;
        T quadrierteDistanz;
//        AABB<T,dim> aabbLeft;
//        AABB<T,dim> aabbRight;
        unsigned char dimSeperation;
        T hilf;
        
        const KdTreeKnoten<T>* node = nodeBegin;
        
        //Taversieren durch Baum bis zu einem Blattknoten
        while(!node->istBlatt()){
            
            const KdTreeElternKnoten<T>* pnode
                                = (const KdTreeElternKnoten<T>*) node;
            
            dimSeperation = pnode->dimSeperation;
//            aabbLeft = aabbNode;
//            aabbRight = aabbNode;
//
//            aabbLeft.max(dimSeperation) = aabbRight.min(dimSeperation) = node->knotenDaten.separator;
//            
//            quadrierteDistanzLeft = aabbLeft.squaredDist(anfragePunkt);
//            quadrierteDistanzRight = aabbRight.squaredDist(anfragePunkt);
            
            hilf = aabbNode.max(dimSeperation);
            aabbNode.max(dimSeperation) = pnode->separator;
            //squared distance to AABB of left child
            quadrierteDistanzLeft = aabbNode.quadrierterAbstand(anfragePunkt);
            aabbNode.max(dimSeperation) = hilf;
            
            hilf = aabbNode.min(dimSeperation);
            aabbNode.min(dimSeperation) = pnode->separator;
            //squared distance to AABB of right child
            quadrierteDistanzRight = aabbNode.quadrierterAbstand(anfragePunkt);
            aabbNode.min(dimSeperation) = hilf;
            
            
            if(quadrierteDistanzLeft < quadrierteDistanzRight){//choose left path
                //nodeQueue.push(SearchEntry(nodeRight,distRight));
                //save right path in queue
                hilf = aabbNode.min(dimSeperation);
                aabbNode.min(dimSeperation) = pnode->separator;
                nodeQueue.push(SearchEntry(pnode->rechts, aabbNode,
                                                quadrierteDistanzRight));
                //nodeQueue.push(SearchEntry(node->knotenDaten.rechts,aabbRight,quadrierteDistanzRight));
                aabbNode.min(dimSeperation) = hilf;
                
                //aabbNode=aabbLeft;
                aabbNode.max(dimSeperation) = pnode->separator;
                node = pnode->links;
            }
            else{//chosse rechts path
                //nodeQueue.push(SearchEntry(nodeLeft, distLeft));
                //save left path in queue
                hilf = aabbNode.max(dimSeperation);
                aabbNode.max(dimSeperation) = pnode->separator;
                nodeQueue.push(SearchEntry(pnode->links, aabbNode,
                                            quadrierteDistanzLeft));
                //nodeQueue.push(SearchEntry(node->knotenDaten.links, aabbLeft, quadrierteDistanzLeft));
                aabbNode.max(dimSeperation) = hilf;
                
                //aabbNode=aabbRight;
                aabbNode.min(dimSeperation) = pnode->separator;
                node = pnode->rechts;
                
            }
        }
        
        const KdTreeBlattKnoten<T>* leafnode = (const KdTreeBlattKnoten<T>*)node;
        //test points
        unsigned int i = leafnode->idxBegin,
                     endi = leafnode->idxEnd;
        for(; i < endi; ++i){
            quadrierteDistanz = anfragePunkt.quadrierteDistanzZu(
                                        *(Tree<T,dim>::_pPointList[i].second));

            if(quadrierteDistanz <= best.quadrierteDistanz){
                best.quadrierteDistanz = quadrierteDistanz;
                //best.dist=sqrt(quadrierteDistanz);
                best.idx = Tree<T, dim>::_pPointList[i].first;
            }
        }
    }

    ////////////////////////////////////
    NachbarDaten<T> nn(const Vek<T, dim>& anfragePunkt) const {
        
        assert(_wurzel != nullptr);
        
        std::priority_queue<SearchEntry> nodeQueue;
        
        AABB<T,dim> aabbNode = _aabb;
        
        T quadrierteDistanz;
        
        //NachbarDaten<T> best(0,std::numeric_limits<T>::max(),std::numeric_limits<T>::max());
        NachbarDaten<T> best(0, 0, std::numeric_limits<T>::max());
        
        const KdTreeKnoten<T>* node;        
        considerPath(aabbNode, nodeQueue, best, _wurzel, anfragePunkt);
        
        while(!nodeQueue.empty()){
            node = nodeQueue.top().node;
            aabbNode = nodeQueue.top().aabbNode;
            quadrierteDistanz = nodeQueue.top().quadrierteDistanz;
            nodeQueue.pop();
            if(quadrierteDistanz > best.quadrierteDistanz) break;
            considerPath(aabbNode, nodeQueue, best, node, anfragePunkt);
        }
        //compute distance -> not needed before
        best.distanz = sqrt(best.quadrierteDistanz);
        return best;
    }
    
    ////////////////////////////////////
    void nn(NachbarDaten<T>& nachbar, const Vek<T,dim>& anfragePunkt) const {
        
        assert(_wurzel != nullptr);
        
        std::priority_queue<SearchEntry> nodeQueue;
        
        AABB<T, dim> aabbNode = _aabb;
        
        T quadrierteDistanz;
       
        if(nachbar.idx < (unsigned int)Tree<T,dim>::_pPointList.size())
            nachbar.quadrierteDistanz = anfragePunkt.quadrierteDistanzZu(
                            *(Tree<T,dim>::_pPointList[nachbar.idx].second));
        else nachbar.quadrierteDistanz = std::numeric_limits<T>::max();
        
        const KdTreeKnoten<T>* node;        
        considerPath(aabbNode, nodeQueue, nachbar, _wurzel, anfragePunkt);
        
        while(!nodeQueue.empty()){
            node = nodeQueue.top().node;
            aabbNode = nodeQueue.top().aabbNode;
            quadrierteDistanz = nodeQueue.top().quadrierteDistanz;
            nodeQueue.pop();
            if(quadrierteDistanz > nachbar.quadrierteDistanz) break;
            considerPath(aabbNode, nodeQueue, nachbar, node, anfragePunkt);
        }
        //compute distance -> not needed before
        nachbar.distanz = sqrt(nachbar.quadrierteDistanz);
    }
    

    //##################################
    ////////////////////////////////////
    void considerPath(
        AABB<T, dim>& aabbNode,
        std::priority_queue<SearchEntry>& nodeQueue,
        std::priority_queue<NachbarDaten<T> >& k_best,
        size_t k,
        const KdTreeKnoten<T>* nodeBegin,
        const Vek<T, dim>& anfragePunkt
    ) const{
        T quadrierteDistanzLeft,quadrierteDistanzRight;
        T quadrierteDistanz;
//        AABB<T,dim> aabbLeft;
//        AABB<T,dim> aabbRight;
        unsigned char dimSeperation;
        T hilf;
        
        const KdTreeKnoten<T>* node = nodeBegin;
        
        while(!node->istBlatt()){
            
            const KdTreeElternKnoten<T>* pnode
                                    = (const KdTreeElternKnoten<T>*)node;
            
            dimSeperation = pnode->dimSeperation;
//            aabbLeft = aabbNode;
//            aabbRight = aabbNode;
//            aabbLeft.max(dimSeperation) = aabbRight.min(dimSeperation) = node->knotenDaten.separator;
//            quadrierteDistanzLeft = aabbLeft.squaredDist(anfragePunkt);
//            quadrierteDistanzRight = aabbRight.squaredDist(anfragePunkt);
            
            hilf = aabbNode.max(dimSeperation);
            aabbNode.max(dimSeperation) = pnode->separator;
            //squared distance to AABB of links child
            quadrierteDistanzLeft = aabbNode.quadrierterAbstand(anfragePunkt);
            aabbNode.max(dimSeperation) = hilf;
            
            hilf = aabbNode.min(dimSeperation);
            aabbNode.min(dimSeperation) = pnode->separator;
            //squared distance to AABB of rechts child
            quadrierteDistanzRight = aabbNode.quadrierterAbstand(anfragePunkt);
            aabbNode.min(dimSeperation) = hilf;
            
            
            if(quadrierteDistanzLeft < quadrierteDistanzRight){//chosse links path
                //nodeQueue.push(SearchEntry(nodeRight, distRight));
                //hilf rechts path in queue
                hilf = aabbNode.min(dimSeperation);
                aabbNode.min(dimSeperation) = pnode->separator;
                nodeQueue.push(SearchEntry(pnode->rechts, aabbNode,
                                                quadrierteDistanzRight));
                //nodeQueue.push(SearchEntry(node->knotenDaten.rechts,aabbRight,quadrierteDistanzRight));
                aabbNode.min(dimSeperation) = hilf;
                
                //aabbNode=aabbLeft;
                aabbNode.max(dimSeperation) = pnode->separator;
                node = pnode->links;
            }
            else{//chosse rechts path
                //nodeQueue.push(SearchEntry(nodeLeft,distLeft));
                //hilf links path in queue
                hilf = aabbNode.max(dimSeperation);
                aabbNode.max(dimSeperation) = pnode->separator;
                nodeQueue.push(
                    SearchEntry(pnode->links, aabbNode, quadrierteDistanzLeft));
                //nodeQueue.push(SearchEntry(node->knotenDaten.links,aabbLeft,quadrierteDistanzLeft));
                aabbNode.max(dimSeperation) = hilf;

                //aabbNode=aabbRight;
                aabbNode.min(dimSeperation) = pnode->separator;                
                node = pnode->rechts;
            }
        }
        
        //test points
        const KdTreeBlattKnoten<T>* leafnode = (const KdTreeBlattKnoten<T>*) node;
        unsigned int i = leafnode->idxBegin, endi = leafnode->idxEnd;
        for(; i < endi; ++i){
            quadrierteDistanz = 
                anfragePunkt.quadrierteDistanzZu(*(Tree<T, dim>::_pPointList[i].second));

            //if better add to list
            if(k_best.size() < k){
                k_best.push(NachbarDaten<T>(Tree<T, dim>::_pPointList[i].first, 0, quadrierteDistanz));
                //k_best.push(NachbarDaten<T>(_idxList[i],sqrt(quadrierteDistanz),quadrierteDistanz));
            }
            else if( k_best.top().quadrierteDistanz > quadrierteDistanz ){
                k_best.pop();
                k_best.push(NachbarDaten<T>(Tree<T, dim>::_pPointList[i].first, 0, quadrierteDistanz));
                //k_best.push(NachbarDaten<T>(_idxList[i],sqrt(quadrierteDistanz),quadrierteDistanz));
            }
        }
    
    }
    

    ////////////////////////////////////
    void knn(
            std::vector<NachbarDaten<T> >& kNachbarn,
            const Vek<T, dim>& anfragePunkt,
            size_t k
    ) const{
        assert(_wurzel != nullptr);
        std::priority_queue<SearchEntry> nodeQueue;
        
        AABB<T, dim> aabbNode = _aabb;
        
        std::priority_queue<NachbarDaten<T> > k_best;
        
        T quadrierteDistanz;
        
        kNachbarn.clear();
        
        const KdTreeKnoten<T>* node;        
        considerPath(aabbNode, nodeQueue, k_best, k, _wurzel, anfragePunkt);
        
        while(!nodeQueue.empty()){
            node = nodeQueue.top().node;
            aabbNode = nodeQueue.top().aabbNode;
            quadrierteDistanz = nodeQueue.top().quadrierteDistanz;
            nodeQueue.pop();
            if(!k_best.empty())
                if(quadrierteDistanz > k_best.top().quadrierteDistanz) break;
            considerPath(aabbNode, nodeQueue, k_best, k, node, anfragePunkt);
        }
        
        //Kopieren der Werte in Nachbarvektor
        kNachbarn.resize(k_best.size());
        for(auto it = kNachbarn.rbegin(), itEnd = kNachbarn.rend(); it != itEnd; ++it){
            *it = k_best.top();
            //compute distance -> not needed before
            it->distanz = sqrt(it->quadrierteDistanz);
            k_best.pop();
        }
        
    }

    
    //########################################
    //durchlaufene Knoten
    //Input:
    //Quellpunkt des Strahls start
    //Richtungsvektor des Strahls rvek
    //Liste mit den durchlaufenen Knoten, wird gefuellt
    void durchlaufeneKnoten(
                const Vek<T, dim> &start,
                const Vek<T, dim> &richtungsVek,
                std::vector<DurchlaufenerKdTreeKnoten<T> >& knotenListe
    ) const {
        
        //ggf. Leeren der Liste
        knotenListe.clear();

        std::stack< std::pair<const KdTreeKnoten<T>*, AABB<T, dim> > > knotenStack;
        
        //Strahlparameter
        T t0, t1;
        
        T hilf;
        unsigned char dimSeperation;
        
        AABB<T, dim> aabbNode;
        
        //Hilfszeiger zum Zwischenspeichern
        const KdTreeKnoten<T>* knoten;
        
        if(!_wurzel->istBlatt())
            knotenStack.push(std::make_pair(_wurzel, _aabb));
        else
            if(_aabb.strahlSchneidetAABB(start, richtungsVek, t0, t1))
                knotenListe.push_back(
                            DurchlaufenerKdTreeKnoten<T>(_wurzel, t0, t1));
        
        while(!knotenStack.empty()){
            //Zwischenspeichern des obersten Eintrags
            knoten = knotenStack.top().first;
            
            //AABB
            aabbNode = knotenStack.top().second;
            
            //Entfernen des obersten Eintrags aus der Liste
            knotenStack.pop();
            //Testen des obersten Eintrags
            if(aabbNode.strahlSchneidetAABB(start, richtungsVek, t0, t1)){
                if(knoten->istBlatt())
                    knotenListe.push_back(
                                DurchlaufenerKdTreeKnoten<T>(knoten, t0, t1));
                else{
                    
                    const KdTreeElternKnoten<T>* elternKnoten =
                        (const KdTreeElternKnoten<T>*) knoten;
                    
                    //Dimension
                    dimSeperation = elternKnoten->dimSeperation;
                    
                    //AABB fuer linken Kindkonten
                    hilf = aabbNode.max(dimSeperation);
                    aabbNode.max(dimSeperation) = elternKnoten->separator;
                    //Hinzufuegen zum Stack
                    knotenStack.push(
                                std::make_pair(elternKnoten->links, aabbNode));
                    aabbNode.max(dimSeperation) = hilf;
                    
                    //AABB fuer rechten Kindkonten
                    aabbNode.min(dimSeperation) = elternKnoten->separator;
                    //Hinzufuegen zum Stack
                    knotenStack.push(
                                std::make_pair(elternKnoten->rechts, aabbNode));
                    //aabbNode.min(dimSeperation) = hilf;
                }
                    //sammleDurchlaufeneKnoten(start, richtungsVek, knotenListe, _wurzel);
            }
        }
        
        std::sort(knotenListe.begin(), knotenListe.end());
    }
    
    
    //###############################################
    void durchlaufeneKnotenIndizesListen(
        const Vek<T, dim>& start,
        const Vek<T, dim>& richtungsVek,
        std::vector< const std::vector<unsigned int>* >& indizesListenZeiger
    ) const {
        indizesListenZeiger.clear();
        std::vector<DurchlaufenerKdTreeKnoten<T> > knotenListe;
        this->durchlaufeneKnoten(start, richtungsVek, knotenListe);
        
        for(auto& kl : knotenListe)
            indizesListenZeiger.push_back(
                        &((const KdTreeBlattKnoten<T>*)kl.knoten)->liste);
    }
    
    //########################################
    //Nachbarn im vorgegebenen Radius
    void nachbarnImUmkreis(
        std::vector<NachbarDaten<T> > &nachbarDaten,
        const Vek<T, dim>& suchPunkt,
        const T& radius
    ) const{
        
        nachbarDaten.clear();
        
        if(_wurzel == nullptr) return;
        
        const KdTreeKnoten<T> *knoten = _wurzel;
        const T quadrierterRadius = radius*radius;
        
        AABB<T,dim> aabbKnoten = _aabb;
        
        unsigned char dimSeperation;
        T hilf;
        
        /////////////////
        //AABB um Kugel:
        AABB<T, dim> aabbKugel;
        
        for(unsigned int i = 0; i < dim; ++i){
            aabbKugel.pmin(i) = suchPunkt(i) - radius;
            aabbKugel.pmax(i) = suchPunkt(i) + radius;
        }
        
        //Knoten durchsuchen
        //Traversieren solange aabbKugel in einer Knoten-AABB liegt
        while(!knoten->istBlatt()){
            
            const KdTreeElternKnoten<T>* elternKnoten
                                = (const KdTreeElternKnoten<T>*)knoten;
            
            //Berechnen der AABB des linken Kindknotens
            dimSeperation = elternKnoten->dimSeperation;
            hilf = aabbKnoten.max(dimSeperation);
            aabbKnoten.max(dimSeperation) = elternKnoten->separator;
            
            if(aabbKugel.inAABB(aabbKnoten))
                knoten = elternKnoten->links;
            else{
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen
                //AABB fuer rechten Kindkonten
                hilf = aabbKnoten.min(dimSeperation);
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabbKugel.inAABB(aabbKnoten))
                    knoten = elternKnoten->rechts;            
                else {
                    aabbKnoten.min(dimSeperation) = hilf;//Wiederherstellen
                    break;
                }
            }
        }

        //Stapel mit Knoten und zugehoeriger AABB zum Abspeichern nicht bearbeiteter Pfade
        std::stack< std::pair<const KdTreeKnoten<T>*,AABB<T,dim> > > knotenStapel;

        knotenStapel.push(std::make_pair(knoten, aabbKnoten));
        
        while(!knotenStapel.empty()){
            //Zwischenspeichern des obersten Eintrags
            knoten = knotenStapel.top().first;
            
            aabbKnoten = knotenStapel.top().second;
            
            //Entfernen des obersten Eintrags aus der Liste
            knotenStapel.pop();
            
            if(knoten->istBlatt()) {
                
                const KdTreeBlattKnoten<T>* blattKnoten
                                        = (const KdTreeBlattKnoten<T>*)knoten;
                T quadrierteDistanz;//,hilf;
                unsigned int i = blattKnoten->idxBegin,
                        endei = blattKnoten->idxEnd;
                for(; i < endei; ++i){
                    //Distanz zwischen Anfragepunkt und Punkt im Knoten
                    quadrierteDistanz = suchPunkt.quadrierteDistanzZu(
                                        *(Tree<T,dim>::_pPointList[i].second));
                    if(quadrierteDistanz <= quadrierterRadius)
                        nachbarDaten.push_back(NachbarDaten<T>(
                            Tree<T,dim>::_pPointList[i].first,
                            sqrt(quadrierteDistanz), quadrierteDistanz));
                }
            }
            //kein Blattknoten:
            else {
                const KdTreeElternKnoten<T>* elternKnoten
                                    = (const KdTreeElternKnoten<T>*) knoten;
                
                dimSeperation = elternKnoten->dimSeperation;
                //AABB des linken Kindknotens
                hilf = aabbKnoten.max(dimSeperation);//Sichern
                aabbKnoten.max(dimSeperation) = elternKnoten->separator;
                //Test auf Schnitt der AABB des Knotens mit Kugel
                if(aabbKnoten.quadrierterAbstand(suchPunkt) <= quadrierterRadius)//aabbKnoten.kugelSchneidetAABB(suchPunkt, quadrierterRadius))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(
                            std::make_pair(elternKnoten->links, aabbKnoten));
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen

                //AABB fuer rechten Kindkonten
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabbKnoten.quadrierterAbstand(suchPunkt) <= quadrierterRadius)//aabbKnoten.kugelSchneidetAABB(suchPunkt, quadrierterRadius))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(
                            std::make_pair(elternKnoten->rechts, aabbKnoten));
            }
        }
        
        //Sortieren, kleinstes Element zuerst
        std::sort(nachbarDaten.begin(), nachbarDaten.end());
    }
    
    
    
    //########################################
    //Funktion zum Einordnen einer AABB in den Baum
    void insertAABB(
        const AABB<T, dim> &aabb,
        unsigned int idx//Index der BBox  
    ){
        if(_wurzel == nullptr) return;
        
        KdTreeKnoten<T> *knoten = _wurzel;
        
        AABB<T,dim> aabbKnoten = _aabb;
        
        unsigned char dimSeperation;
        T hilf;
        
        //Knoten durchsuchen
        while(!knoten->istBlatt()){
            
            const KdTreeElternKnoten<T>* elternKnoten
                                    = (const KdTreeElternKnoten<T>*)knoten;
            
            //Berechnen der AABB des linken Kindknotens
            dimSeperation = elternKnoten->dimSeperation;
            hilf = aabbKnoten.max(dimSeperation);//Sichern
            aabbKnoten.max(dimSeperation) = elternKnoten->separator;
            
            if(aabb.inAABB(aabbKnoten))
                knoten = elternKnoten->links;
            else{
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen
                //AABB fuer rechten Kindkonten
                hilf = aabbKnoten.min(dimSeperation);//Sichern
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabb.inAABB(aabbKnoten))
                    knoten = elternKnoten->rechts;            
                else {
                    aabbKnoten.min(dimSeperation) = hilf;//Wiederherstellen
                    break;
                }
            }
        }
        
        //Stapel fuer Knoten, die noch bearbeitet werden muessen
        std::stack< std::pair<KdTreeKnoten<T>*,AABB<T, dim> > > knotenStapel;

        knotenStapel.push(std::make_pair(knoten,aabbKnoten));
        
        while(!knotenStapel.empty()){
            //Zwischenspeichern des obersten Eintrags
            knoten = knotenStapel.top().first;
            
            aabbKnoten = knotenStapel.top().second;
            
            //Entfernen des obersten Eintrags aus der Liste
            knotenStapel.pop();
            //Testen des obersten Eintrags
            if(knoten->istBlatt())
                //knoten->blattDaten.liste.push_back(idx);
                ((KdTreeBlattKnoten<T>*)knoten)->liste.push_back(idx);
            else{
                
                KdTreeElternKnoten<T>* elternKnoten
                                            = (KdTreeElternKnoten<T>*) knoten;
                
                dimSeperation = elternKnoten->dimSeperation;
                //AABB des linken Kindknotens
                hilf = aabbKnoten.max(dimSeperation);
                aabbKnoten.max(dimSeperation) = elternKnoten->separator;
                //Hinzufuegen zum Stack, wenn die einzuordnende AABB die AABB des Knotens schneidet
                if(aabb.schneidetAABB(aabbKnoten))
                    knotenStapel.push(
                            std::make_pair(elternKnoten->links, aabbKnoten));
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen

                //AABB fuer rechten Kindkonten
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                //Hinzufuegen zum Stack, wenn die einzuordnende AABB die AABB des Knotens schneidet
                if(aabb.schneidetAABB(aabbKnoten))
                    knotenStapel.push(
                            std::make_pair(elternKnoten->rechts, aabbKnoten));
            }
        }   
    }
    
    
    
    //########################################
    //Nachbarn in vorgegebener AABB
    void nachbarnInAABB(
        std::vector<unsigned int> &nachbarIndizes,
        const AABB<T, dim>& aabb
    ) const{
        
        nachbarIndizes.clear();
        
        if(_wurzel == nullptr) return;
        
        const KdTreeKnoten<T> *knoten = _wurzel;
        
        AABB<T, dim> aabbKnoten = _aabb;
        
        unsigned char dimSeperation;
        T hilf;
        
        //Knoten durchsuchen
        while(!knoten->istBlatt()){
            
            const KdTreeElternKnoten<T>* elternKnoten
                            = (const KdTreeElternKnoten<T>*) knoten;
            
            //Berechnen der AABB des linken Kindknotens
            dimSeperation = elternKnoten->dimSeperation;
            hilf = aabbKnoten.max(dimSeperation);
            aabbKnoten.max(dimSeperation) = elternKnoten->separator;
            
            if(aabb.inAABB(aabbKnoten))
                knoten = elternKnoten->links;
            else{
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen
                //AABB fuer rechten Kindkonten
                hilf = aabbKnoten.min(dimSeperation);
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabb.inAABB(aabbKnoten))
                    knoten = elternKnoten->rechts;
                else {
                    aabbKnoten.min(dimSeperation) = hilf;//Wiederherstellen
                    break;
                }
            }
        }

        std::stack<std::pair<const KdTreeKnoten<T>*, AABB<T,dim> > > knotenStapel;

        knotenStapel.push(std::make_pair(knoten,aabbKnoten));
        
        while(!knotenStapel.empty()){
            //Zwischenspeichern des obersten Eintrags
            knoten = knotenStapel.top().first;
            
            aabbKnoten = knotenStapel.top().second;
            
            //Entfernen des obersten Eintrags aus der Liste
            knotenStapel.pop();
            
            if(knoten->istBlatt()){
                const KdTreeBlattKnoten<T> *blattKnoten
                                        = (const KdTreeBlattKnoten<T>*) knoten;
//                T quadrierteDistanz;//,hilf;
                unsigned int i = blattKnoten->idxBegin,
                        endei = blattKnoten->idxEnd;
                for(; i < endei; ++i){
                    if(aabb.punktInAABB(*(Tree<T, dim>::_pPointList[i].second)))
                        nachbarIndizes.push_back(
                                Tree<T, dim>::_pPointList[i].first);
                }
            }
            //kein Blattknoten:
            else{
                const KdTreeElternKnoten<T>* elternKnoten
                                    = (const KdTreeElternKnoten<T>*)knoten;
                
                dimSeperation = elternKnoten->dimSeperation;
                //AABB des linken Kindknotens
                hilf = aabbKnoten.max(dimSeperation);//Sichern
                aabbKnoten.max(dimSeperation) = elternKnoten->separator;
                if(aabbKnoten.schneidetAABB(aabb))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(
                            std::make_pair(elternKnoten->links, aabbKnoten));
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen

                //AABB fuer rechten Kindkonten
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabbKnoten.schneidetAABB(aabb))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(std::make_pair(elternKnoten->rechts,
                                                    aabbKnoten));
            }
            
        }
        
        //Sortieren, kleinstes Element zuerst
//        std::sort(nachbarIndizes.begin(),nachbarIndizes.end());
    }
    
    
    //########################################
    //Nachbarn im vorgegebenen Radius
    void nachbarnInAABB(
        std::vector<NachbarDaten<T> > &nachbarDaten,
        const Vek<T, dim>& suchPunkt,
        const T& radius
    ) const {
        
        nachbarDaten.clear();
        
        if(_wurzel == nullptr) return;
        
        const KdTreeKnoten<T> *knoten = _wurzel;
        //const T quadrierterRadius = radius*radius;
        
        AABB<T,dim> aabbKnoten = _aabb;
        
        unsigned char dimSeperation;
        T hilf;
        
        /////////////////
        //AABB um Punkt:
        AABB<T, dim> aabbSuch;
        
        for(unsigned int i = 0; i < dim; ++i){
            aabbSuch.pmin(i) = suchPunkt(i) - radius;
            aabbSuch.pmax(i) = suchPunkt(i) + radius;
        }
        
        //Knoten durchsuchen
        while(!knoten->istBlatt()){
            const KdTreeElternKnoten<T>* elternKnoten = (const KdTreeElternKnoten<T>*)knoten;
            //Berechnen der AABB des linken Kindknotens
            dimSeperation = elternKnoten->dimSeperation;
            hilf = aabbKnoten.max(dimSeperation);
            aabbKnoten.max(dimSeperation) = elternKnoten->separator;
            
            if(aabbSuch.inAABB(aabbKnoten))
                knoten = elternKnoten->links;
            else{
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen
                //AABB fuer rechten Kindkonten
                hilf = aabbKnoten.min(dimSeperation);
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabbSuch.inAABB(aabbKnoten))
                    knoten = elternKnoten->rechts;
                else {
                    aabbKnoten.min(dimSeperation) = hilf;//Wiederherstellen
                    break;
                }
            }
        }

        std::stack< std::pair<const KdTreeKnoten<T>*,AABB<T,dim> > > knotenStapel;

        knotenStapel.push(std::make_pair(knoten,aabbKnoten));
        
        while(!knotenStapel.empty()){
            //Zwischenspeichern des obersten Eintrags
            knoten = knotenStapel.top().first;
            
            aabbKnoten = knotenStapel.top().second;
            
            //Entfernen des obersten Eintrags aus der Liste
            knotenStapel.pop();
            
            if(knoten->istBlatt()){
                const KdTreeBlattKnoten<T>* blattKnoten = (const KdTreeBlattKnoten<T>*)knoten;
                T quadrierteDistanz;//,hilf;
                unsigned int i = blattKnoten->idxBegin,
                        endei = blattKnoten->idxEnd;
                for(; i < endei; ++i){
                    if(aabbSuch.punktInAABB(*(Tree<T,dim>::_pPointList[i].second))){
                        //Distanz zwischen Anfragepunkt und Punkt im Knoten
                        quadrierteDistanz = suchPunkt.quadrierteDistanzZu(*(Tree<T,dim>::_pPointList[i].second));
                        nachbarDaten.push_back(NachbarDaten<T>(Tree<T,dim>::_pPointList[i].first,sqrt(quadrierteDistanz),quadrierteDistanz));
                    }
                }
            }
            //kein Blattknoten:
            else{
                const KdTreeElternKnoten<T>* elternKnoten = (const KdTreeElternKnoten<T>*)knoten;
                dimSeperation = elternKnoten->dimSeperation;
                //AABB des linken Kindknotens
                hilf = aabbKnoten.max(dimSeperation);//Sichern
                aabbKnoten.max(dimSeperation) = elternKnoten->separator;
                if(aabbKnoten.schneidetAABB(aabbSuch))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(std::make_pair(elternKnoten->links,aabbKnoten));
                aabbKnoten.max(dimSeperation) = hilf;//Wiederherstellen

                //AABB fuer rechten Kindkonten
                aabbKnoten.min(dimSeperation) = elternKnoten->separator;
                if(aabbKnoten.schneidetAABB(aabbSuch))
                    //Hinzufuegen zum Stack
                    knotenStapel.push(std::make_pair(elternKnoten->rechts,aabbKnoten));
            }
            
        }
        
        //Sortieren, kleinstes Element zuerst
        std::sort(nachbarDaten.begin(), nachbarDaten.end());
    }
    
    
//    //Tiefe, bei der Knoten auf Blattknoten gesetzt wird
//    void setMaxTiefe(unsigned int maxTiefe){
//        Tree<T, dim>::_maxTiefe = maxTiefe;
//    }
//    
//    //Anzahl der Punkte bei der Knoten auf Blattknoten gesetzt wird
//    void setAnzahlPunkteAbbruch(unsigned int anzahl){
//        Tree<T, dim>::_anzahlPunkteAbbruch = anzahl;
//    }
    
    inline bool istInitialisiert() const{
        return _wurzel != nullptr;
    }
    
    
    inline void clear(){
        Tree<T,dim>::_pPointList.clear();
        if(_wurzel != nullptr) delete _wurzel;
        _wurzel = nullptr;
    }
    
};


template<class T>
using KdTree3 = KdTree<T, 3>;

template<class T>
using KdTree2 = KdTree<T, 2>;

typedef KdTree<double, 3> KdTree3d;
typedef KdTree<float, 3> KdTree3f;

typedef KdTree<double, 2> KdTree2d;
typedef KdTree<float, 2> KdTree2f;


#endif //KDTREE_H
