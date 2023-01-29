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
        route(const point<T> &start)
            : std::deque<heading<T, Q> >(),
            __location(start)
        { }

        travel_result<T, Q> follow(const Q duration)
        {
            travel_result<T, Q> result;
            if(this->empty())
            {
                result.target = this->__location;
                result.time_to_waypoint = 0;
                result.location = this->__location;
                result.elapsed_time = 0;
                result.unused_time = duration;
                return result;
            }

            result = this->front().approach(this->__location, duration);
            this->__location = result.location;
            if(result.target_reached())
            {
                this->pop_front();
                result = this->follow(result.unused_time);
            }
            return result;
        }

        const point<T> & location() const { return this->__location; }
    private:
        point<T> __location;
    };
}

#endif