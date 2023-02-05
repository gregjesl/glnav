#ifndef GLNAV_ROUTE_H
#define GLNAV_ROUTE_H

#include "glnav_neighbor.h"
#include "glnav_network.h"
#include "glnav_heading.h"
#include "glnav_version_control.h"
#include <deque>

namespace glnav
{
    template<typename T, typename Q>
    class route : public std::deque<heading<T, Q> >
    {
    public:
        route()
            : std::deque<heading<T, Q> >()
        { }

        route(const route<T, Q> & other)
            : std::deque<heading<T, Q> >(other)
        { }

        route& operator=(const route<T, Q> & other)
        {
            std::deque<heading<T, Q> >::operator=(other);
            return *this;
        }

        const point<T> & target() const 
        { 
            if(this->empty()) throw std::domain_error("No target set");
            return this->back().target; 
        }

        travel_result<T, Q> follow(const point<T> & start, const Q duration)
        {
            if(this->empty())
                return travel_result<T, Q>::idle(start, duration);
            
            travel_result<T, Q> result = this->front().approach(start, duration);
            if(result.target_reached())
            {
                const point<T> next = result.location;
                const Q elapsed_time = result.elapsed_time;
                this->pop_front();
                if(!this->empty())
                {
                    result = this->follow(next, result.unused_time);
                    result.elapsed_time += elapsed_time;
                }
            }
            return result;
        }

        bool traverses(const network<T, Q> &net) const
        {
            if(this->empty()) return false;
            typename std::deque<heading<T, Q> >::const_iterator it, last;
            last = this->begin();
            for(it = this->begin(); it != this->end(); ++it)
            {
                if(!net.contains(it->target)) return false;
                if(it != last && !net.contains(path<T>(last->target, it->target))) return false;
                last = it;
            }
            return true;
        }
    };
}

#endif