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

        travel_result<T, Q> follow(const point<T> &start, const Q duration)
        {
            travel_result<T, Q> result;
            if(this->empty())
            {
                result.time_to_waypoint = 0;
                result.location = start;
                result.elapsed_time = 0;
                result.unused_time = duration;
                return result;
            }

            result = this->front().approach(start, duration);
            if(result.target_reached())
            {
                this->pop_front();
                return this->follow(result.location, result.unused_time);
            }
            return result;
        }
    };
}

#endif