#ifndef GLNAV_CORNER_H
#define GLNAV_CORNER_H

#include "glnav_point.h"
#include "glnav_obstacle_interface.h"

namespace glnav
{
    template<typename T>
    class corner : virtual public obstacle_interface<T>
    {
    public:
        corner(const point<T> &center, const point<T> &leg1, const point<T> &leg2, const point<T> &outside)
            : __center(center), 
            __leg1(leg1 - center), 
            __leg2(leg2 - center),
            __leg1_cross_leg2(this->__leg1.cross(this->__leg2)),
            __leg2_cross_leg1(this->__leg2.cross(this->__leg1)),
            __concave(outside.is_between(center, leg1, leg2))
        {
            if(this->__leg1.cross(this->__leg2) == 0) throw std::invalid_argument("Is not a corner");
            if(center == outside) throw std::invalid_argument("Outside reference is not valid");
        }

        virtual bool obstructs(const path<T> &input) const
        {
            assert(!input.is_point());

            // The most common use case is a path starting or ending on the corner
            if(input.start == this->__center || input.end == this->__center)
            {
                const point<T> test = input.start == this->__center ? input.end - this->__center : input.start - this->__center;

                // Check for parallel paths
                if(test.cross(this->__leg1) == 0 && test.dot(this->__leg1) > 0)
                    return false;
                if(test.cross(this->__leg2) == 0 && test.dot(this->__leg2) > 0)
                    return false;

                const bool is_inside = this->__leg1.cross(test) * this->__leg1_cross_leg2 >= 0
                    && this->__leg2.cross(test) * this->__leg2_cross_leg1 >= 0;

                // If this is concave, then the end should
                return is_inside != this->__concave;
            }

            // The less common case is the path crosses over the point
            if(!input.contains_point(this->__center, false)) return false;
            
            // Recursive check
            return this->obstructs(glnav::path<T>(input.start, this->__center))
                || this->obstructs(glnav::path<T>(this->__center, input.end));
        }

        virtual std::vector<point<T> > corners() const
        {
            std::vector<point<T> > result;
            result.push_back(this->__center);
            return result;
        }
    private:
        const point<T> __center;
        const point<T> __leg1;
        const point<T> __leg2;
        const T __leg1_cross_leg2;
        const T __leg2_cross_leg1;
        const bool __concave;
    };
}

#endif