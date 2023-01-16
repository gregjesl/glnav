#ifndef GLNAV_WAYPOINT_H
#define GLNAV_WAYPOINT_H

#include "glnav_point.h"

namespace glnav
{
    template<typename T>
    class waypoint : public point<T>
    {
    public:
        waypoint(const T x, const T y)
            : point<T>(x, y),
            speed(1)
        { }

        waypoint(const T x, const T y, const double speed)
            : point<T>(x, y),
            speed(speed)
        { }

        double speed;
    };
}

#endif