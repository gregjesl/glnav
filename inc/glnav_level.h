#ifndef GLNAV_LEVEL_H
#define GLNAV_LEVEL_H

#include "glnav_point.h"
#include "glnav_obstacle_interface.h"

namespace glnav
{
    template<typename T>
    class level
    {
    public:
        void navigate(const point<T> &from, const point<T> &to)
        {
            (void)from;
            (void)to;
            // Step 1: Find all potential obstacles

            // Step 2: Build the network

            // Step 3: Translate the network such that the goal is to navigate to (0,0)

            // Step 4: A*
        }
    };
}

#endif