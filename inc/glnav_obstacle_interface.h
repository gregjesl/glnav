#ifndef GLNAV_OBSTACLE_INTERFACE_H
#define GLNAV_OBSTACLE_INTERFACE_H

#include "glnav_point.h"
#include "glnav_path.h"
#include <vector>

namespace glnav
{
    template<typename T>
    class obstacle_interface
    {
    public:
        virtual bool obstructs(const path<T> &input) const = 0;
        virtual point_group<T> corners() const = 0;
    };
}

#endif