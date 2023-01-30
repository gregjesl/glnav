#ifndef GLNAV_DISPATCH_H
#define GLNAV_DISPATCH_H

#include "con3/con3.h"
#include "glnav_traveler.h"
#include "glnav_version_control.h"
#include "glnav_network.h"

namespace glnav
{
    template<typename T, typename Q>
    class dispatch : public con3::set<traveler<T, Q> *>
    {
    public:
        dispatch()
        { }

        void run(const Q duration)
        {
            for(size_t i = 0; i < this->values().size(); i++)
                this->at(i)->travel(duration);
        }
    };
}

#endif