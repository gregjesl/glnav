#ifndef GLNAV_CARTESIAN_H
#define GLNAV_CARTESIAN_H

#include "glnav_point.h"
#include <set>
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
                // Is a point
                if(input.x == this->minX() && input.y == this->minY()) return include_border;
                return false;
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
        cartesian_area()
            : __minX(0), __minY(0), __maxX(0), __maxY(0)
        { }

        cartesian_area(const T x1, const T y1, const T x2, const T y2)
            : __minX(x1 < x2 ? x1 : x2),
            __minY(y1 < y2 ? y1 : y2),
            __maxX(x1 > x2 ? x1 : x2),
            __maxY(y1 > y2 ? y1 : y2)
        { }

        static cartesian_area<T> from_set(const std::set<point<T> > & input)
        {
            cartesian_area<T> result;
            typename std::set<point<T> >::const_iterator it;
            for(it = input.begin(); it != input.end(); it++)
            {
                result.expand(*it);
            }
            return result;
        }

        void expand(const point<T> &input)
        {
            if(input.x < this->__minX)
                this->__minX = input.x;
            else if(input.x > this->__maxX)
                this->__maxX = input.x;

            if(input.y < this->__minY)
                this->__minY = input.y;
            else if(input.y > this->__maxY)
                this->__maxY = input.y;
        }
        
        cartesian_area(const cartesian_object<T> &other)
            : __minX(other.minX()),
            __minY(other.minY()),
            __maxX(other.maxX()),
            __maxY(other.maxY())
        { 
            assert(this->minX() <= this->maxX());
            assert(this->minY() <= this->maxY());
            assert(other.minX() <= other.maxX());
            assert(other.minY() <= other.maxY());
        }

        cartesian_area& operator= (const cartesian_object<T> &other)
        {
            this->__minX = other.minX();
            this->__minY = other.minY();
            this->__maxX = other.maxX();
            this->__maxY = other.maxY();

            assert(this->minX() <= this->maxX());
            assert(this->minY() <= this->maxY());
        }

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