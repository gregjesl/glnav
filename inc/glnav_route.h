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
    };
}

#endif