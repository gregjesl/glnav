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
            // Check for the case of a line
            const bool xline = this->minX() == this->maxX();
            const bool yline = this->minY() == this->maxY();

            // Most common case: neither are a line
            if(!xline && !yline)
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

            if(xline && yline)
            {
                
            }
            
            if(xline && !yline)
            {
                if(input.x != this->minX()) return false;
                return include_border ?
                    input.y >= this->minY()
                    && input.y <= this->maxY()
                    :
                    input.y > this->minY()
                    && input.y < this->maxY();
            }

            if(yline)
            {
                assert(!xline);
                if(input.y != this->minY()) return false;
                return include_border ?
                    input.x >= this->minX()
                    && input.x <= this->maxX()
                    :
                    input.x > this->minX()
                    && input.x < this->maxX();
            }

            // Point
            assert(this->minX() == this->maxX());
            assert(this->minY() == this->maxY());
            return input.x == this->minX()
                && input.y >= this->minY();
        }
    };

    template<typename T>
    class cartesian_area : virtual public cartesian_object<T>
    {
    public:
        cartesian_area(const T x1, const T x2, const T y1, const T y2)
            : __minX(x1 < x2 ? x1 : x2),
            __minY(y1 < y2 ? y1 : y2),
            __maxX(x1 > x2 ? x1 : x2),
            __maxY(y1 > y2 ? y1 : y2)
        { }
        virtual T minX() const { return this->__minX; }
        virtual T maxX() const { return this->__maxX; }
        virtual T minY() const { return this->__minY; }
        virtual T maxY() const { return this->__maxY; }
    private:
        const T __minX;
        const T __minY;
        const T __maxX;
        const T __maxY;
    };
}

#endif