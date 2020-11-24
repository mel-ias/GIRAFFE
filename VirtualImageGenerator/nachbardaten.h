///Frank Liebold

#ifndef NACHBARDATEN_H
#define	NACHBARDATEN_H

template <class T>
class NachbarDaten{
public:
    unsigned int idx;
    T distanz;//Abstand
    T quadrierteDistanz;//Abstandsquadrat

    //Konstruktor
    NachbarDaten() : idx(std::numeric_limits<unsigned int>::max()),
            distanz(std::numeric_limits<T>::max()),
            quadrierteDistanz(std::numeric_limits<T>::max()){}
    //Konstruktor 2
    NachbarDaten(unsigned int index, T distanz, T quadrierteDistanz)
     : idx(index), distanz(distanz), quadrierteDistanz(quadrierteDistanz){}
    
    inline bool operator<(const NachbarDaten<T>& n) const{
        return quadrierteDistanz < n.quadrierteDistanz;
    }
};

typedef NachbarDaten<double> NachbarDatend;
typedef NachbarDaten<float> NachbarDatenf;

#endif