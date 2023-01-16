#ifndef GLNAV_VECTOR_H
#define GLNAV_VECTOR_H

#include "glnav_point.h"
#include <vector>

namespace glnav
{
    template<typename T>
    class vector
    {
    public:
        

    };

    template<typename T>
    class vector_group
    {
    public:
        point<T> center;
        std::vector<point<T> > vectors;
    };
}

#endif