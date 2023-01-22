#ifndef GLNAV_COST_MAP_H
#define GLNAV_COST_MAP_H

#include "glnav_point.h"
#include <map>

namespace glnav
{
    template<typename T>
    class cost_map : public std::map<point<T> , double>
    {
    public:
        cost_map() 
            : std::map<point<T> , double>(),
            __lowest_cost(this->end())
        { }
        
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
            if(this->__lowest_cost == this->end())
                this->__lowest_cost = this->begin();
        }

        void update(const point<T> &input, const double cost)
        {
            typename std::map<point<T> , double>::iterator it = this->find(input);
            if(it == this->end())
            {
                const std::pair<
                    typename std::map<point<T>, double>::const_iterator,
                    bool
                > result = this->insert(
                    std::pair<point<T>, double>
                        (input, cost)
                );
                if(cost < this->__lowest_cost->second && result.second) {
                    this->__lowest_cost = result.first;
                }
            }
            else
            {
                it->second = cost;
                if(cost < this->__lowest_cost->second)
                {
                    this->__lowest_cost = it;
                }
            }
        }

        std::vector<const point<T> *> pointers() const
        {
            std::vector<const point<T> *> result;
            typename std::map<point<T> , double>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                result.push_back(&(it->first));
            }
            return result;
        }

        double lowest_cost() const 
        { 
            if(this->__lowest_cost == this->end())
                return std::numeric_limits<double>::infinity();
            return this->__lowest_cost->second;
        }

        const point<T> * lowest_cost_point() const 
        { 
            return this->__lowest_cost == this->end() ? 
                nullptr
                :
                &this->__lowest_cost->first;
        }

    private:
        typename std::map<point<T> , double>::const_iterator __lowest_cost;
    };
}

#endif