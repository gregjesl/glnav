#ifndef GLNAV_TRAVELER_H
#define GLNAV_TRAVELER_H

#include "glnav_point.h"
#include "glnav_route.h"

namespace glnav
{
    template<typename T, typename Q>
    class traveler
    {
    public:
        traveler(const point<T> &start)
            : __location(start)
        { }

        traveler(const point<T> &start, const route<T, Q> &seed)
            : __location(start),
            __route(seed)
        { }

        traveler& operator=(const traveler<T, Q> & other)
        {
            this->__location = other.__location;
            this->__route = other.__route;
            return *this;
        }

        void teleport(const point<T> & location)
        {
            this->__location = location;
        }

        void set(const route<T, Q> & input)
        {
            this->__route = input;
        }

        const point<T> & location() const { return this->__location; }

        bool is_moving() const { return !this->__route.empty(); }

        travel_result<T, Q> travel(const Q duration)
        {
            if(!this->is_moving()) 
                return travel_result<T, Q>::idle(this->__location, duration);
            
            travel_result<T, Q> result = this->__route.follow(this->__location, duration);
            this->__location = result.location;
            return result;
        }

    private:
        point<T> __location;
        route<T, Q> __route;
    };
}

#endif