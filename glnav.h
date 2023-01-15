#ifndef GLNAV_H
#define GLNAV_H

#include "inc/glnav_point.h"
#include "inc/glnav_pin.h"
#include "inc/glnav_path.h"
#include "inc/glnav_fence.h"

#include <vector>
#include <list>
#include <stdexcept>
#include <assert.h>
#include <math.h>

namespace glnav
{

    template<typename T>
    class obstacle
    {

    };

    template<typename T>
    class route : public std::list<point<T> >
    {

    };
}

#endif