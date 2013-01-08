#include <QHash>
#include <tr1/functional>

struct DoubleTuple{
    double x, y, z;
    int index;

    DoubleTuple(double X = 0, double Y = 0, double Z = 0, int Index = -1){
        x = X; y = Y; z = Z; index = Index;
    }

    bool operator== (const DoubleTuple & other) const{
        return this->x == other.x && this->y == other.y && this->z == other.z;
    }
};

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::tr1::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}

struct DoubleTupleMap{

    void insert(double x, double y, double z, int index = -1)
    {
        size_t key = hash(x,y,z);

        map[ key ].push_back( DoubleTuple(x,y,z, index) );
    }

    int count(double x, double y, double z)
    {
        size_t key = hash(x,y,z);
        if(!map.contains( key )) return 0;

        return map[ key ].count( DoubleTuple(x,y,z, -1) );
    }

    int firstIndex(double x, double y, double z)
    {
        QList<DoubleTuple> l = map[ hash(x,y,z) ];

        return l.takeAt( l.indexOf(DoubleTuple(x,y,z,-1)) ).index;
    }

    inline size_t hash(double x, double y, double z){
        size_t seed = 0;

        hash_combine(seed, x);
        hash_combine(seed, y);
        hash_combine(seed, z);

        return seed;
    }

    QHash< size_t, QList<DoubleTuple> > map;
};
