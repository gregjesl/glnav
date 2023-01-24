#ifndef GLNAV_NEIGHBOR_H
#define GLNAV_NEIGHBOR_H

#include "glnav_point.h"
#include "glnav_node.h"
#include <vector>

namespace glnav
{
    template<typename T, typename Q>
    class neighbor : public node<T>
    {
    public:
        neighbor(const point<T> &location, const Q cost)
            : node<T>(location), 
            cost(cost)
        { }

        neighbor(const neighbor &other)
            : node<T>(other),
            cost(other.cost)
        { }

        neighbor& operator=(neighbor &other)
        {
            node<T>::operator=(other);
            this->cost = other.cost;
            return *this;
        }

        Q cost;
    };

    template<typename T, typename Q>
    class neighborhood : public std::vector<neighbor<T, Q> >
    {
    public:
        neighborhood()
            : std::vector<neighbor<T, Q> >()
        { }

        neighborhood(const neighborhood &other)
            : std::vector<neighbor<T, Q> >(other)
        { }

        neighborhood& operator=(const neighborhood &other)
        {
            std::vector<neighbor<T, Q> >::operator=(other);
            return *this;
        }

        bool contains(const point<T> &location) const
        {
            for(size_t i = 0; i < this->size(); i++)
            {
                if(this->at(i).is_located_at(location)) return true;
            }
            return false;
        }

        Q cost(const point<T> &location) const
        {
            for(size_t i = 0; i < this->size(); i++)
            {
                if(this->at(i).is_located_at(location)) return this->at(i).cost;
            }
            throw glnav::unknown_point_exception<T>(location);
        }
    };
}


#endif