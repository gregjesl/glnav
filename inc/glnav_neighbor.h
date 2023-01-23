#ifndef GLNAV_NEIGHBOR_H
#define GLNAV_NEIGHBOR_H

#include "glnav_point.h"

namespace glnav
{
    template<typename T, typename Q>
    class neighbor : public point<T>
    {
    public:
        neighbor(const point<T> &location, const Q cost)
            : point<T>(location),
            cost(cost)
        { }

        neighbor(const neighbor &other)
            : point<T>(other),
            cost(other.cost)
        { }

        neighbor& operator=(const neighbor &other)
        {
            point<T>::operator=(other);
            this->cost = other.cost;
            return *this;
        }

        bool is_located_at(const point<T> &location) const
        {
            return point<T>::operator=(location);
        }

        Q cost;
    };

    template<typename T, typename Q>
    class neighborhood : public std::vector<neighbor<T, Q> >
    {
    public:
        neighborhood(const point<T> &origin)
            : std::vector<neighbor<T, Q> >(),
            __origin(origin)
        { }

        neighborhood(const neighborhood &other)
            : std::vector<neighbor<T, Q> >(other),
            __origin(other.__origin)
        { }

        neighborhood& operator=(const neighborhood &other)
        {
            std::vector<neighbor<T, Q> >::operator=(other);
            this->__origin = other.__origin;
            return *this;
        }

        /*! \warning The origin is not included in the search */
        bool contains(const point<T> &location)
        {
            for(size_t i = 0; i < this->size(); i++)
            {
                if(this->at(i).is_located_at(location)) return true;
            }
            return false;
        }
    private:
        point<T> __origin;
    };
}


#endif