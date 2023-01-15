#ifndef GLNAV_CARTESIAN_H
#define GLNAV_CARTESIAN_H

#include "glnav_point.h"
#include <assert.h>

namespace glnav
{
    template<typename T>
    class cartesian_object
    {
    public:
        virtual T minX() const = 0;
        virtual T maxX() const = 0;
        virtual T minY() const = 0;
        virtual T maxY() const = 0;

        bool could_overlap(const cartesian_object &other, const bool include_border) const
        {
            assert(this->minX() <= this->maxX());
            assert(this->minY() <= this->maxY());
            assert(other.minX() <= other.maxX());
            assert(other.minY() <= other.maxY());

            if(include_border)
            {
                if(this->minX() > other.maxX()) return false;
                if(this->maxX() < other.minX()) return false;
                if(this->minY() > other.maxY()) return false;
                if(this->maxY() < other.minY()) return false;
            }
            else
            {
                if(this->minX() >= other.maxX()) return false;
                if(this->maxX() <= other.minX()) return false;
                if(this->minY() >= other.maxY()) return false;
                if(this->maxY() <= other.minY()) return false;
            }
            return true;
        }

        bool could_contain(const point<T> &input, const bool include_border) const
        {
            return include_border ?
                input.x >= this->minX()
                && input.x <= this->maxX()
                && input.y >= this->minY()
                && input.y <= this->maxY()
                :
                input.x > this->minX()
                && input.x < this->maxX()
                && input.y > this->minY()
                && input.y < this->maxY();
        }
    };
}

#endif