#ifndef GLNAV_COST_MAP_H
#define GLNAV_COST_MAP_H

#include "glnav_point.h"
#include "glnav_network.h"
#include <map>

namespace glnav
{
    template<typename T>
    class cost_map : public std::map<point<T> , double>
    {
    public:
        cost_map() 
            : std::map<point<T> , double>(),
            __seed(0)
        { }

        cost_map(const network<T> &net)
            : std::map<point<T> , double>(net.node_map(std::numeric_limits<double>::infinity())),
            __seed(net.version())
        { }

        cost_map& operator=(const cost_map &other)
        {
            std::map<point<T> , double>::operator=(other);
            this->__seed = other.__seed;
            return *this;
        }
        
        double cost(const point<T> &input) const
        {
            typename std::map<point<T> , double>::const_iterator it = this->find(input);
            if(it != this->end())
            {
                return it->second;
            }
            return std::numeric_limits<double>::infinity();
        }

        void seed(const point<T> &input)
        {
            this->insert(
                std::pair<point<T>, double>
                    (input, std::numeric_limits<double>::infinity())
                );
        }

        void update(const point<T> &input, const double cost)
        {
            typename std::map<point<T> , double>::iterator it = this->find(input);
            if(it == this->end())
            {
                this->insert(
                    std::pair<point<T>, double>
                        (input, cost)
                );
            }
            else
            {
                it->second = cost;
            }
        }

        void update(const cost_map<T> &from)
        {
            typename std::map<point<T> , double>::const_iterator it;
            for(it = from.begin(); it != from.end(); ++it)
            {
                this->update(it->first, it->second);
            }
        }
    private:
        version_t __seed;
    };
}

#endif