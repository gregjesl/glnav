#ifndef GLNAV_ROUTE_H
#define GLNAV_ROUTE_H

#include "glnav_waypoint.h"
#include "glnav_cartesian.h"
#include <list>

namespace glnav
{
    template<typename T>
    class route : virtual public cartesian_object<T>, private std::list<waypoint<T> >
    {
    public:
        typedef std::list<waypoint<T> > waypoints_t;
        route(const waypoint &seed)
            : cartesian_object<T>(),
            waypoints_t()
        { }
        virtual T minX() const { return this->__minX; }
        virtual T maxX() const { return this->__maxX; }
        virtual T minY() const { return this->__minX; }
        virtual T maxY() const { return this->__maxX; }
    private:
        T __minX;
        T __minY;
        T __maxX;
        T __maxY;

        void __update_min(const T input, T &value)
        {
            if(input < value) input = value;
        }

        void __update_max(const T input, T &value)
        {
            if(input > value) input = value;
        }

        void __update_axis(const T value, T &min, T &max)
        {
            this->__update_min(value, min);
            this->__update_max(value, max);
        }
    };
}

#endif