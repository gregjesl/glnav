#ifndef GLNAV_PIN_H
#define GLNAV_PIN_H

#include "glnav_point.h"
#include <stdexcept>
#include <string>

namespace glnav
{
    /*! \brief Class for handling a point that has sides */
    template<typename T>
    class pin : public point<T>
    {
    public:
        pin(const point<T> &center, const point<T> &leg1, const point<T> &leg2)
            : point<T>(center)
        {
            this->leg[0] = leg1;
            this->leg[1] = leg2;
        }

        pin(const T centerX, const T centerY, const T leg1x, const T leg1y, const T leg2x, const T leg2y)
            : point<T>(centerX, centerY)
        {
            this->leg[0] = point<T>(leg1x, leg1y);
            this->leg[1] = point<T>(leg2x, leg2y);
        }

        point<T> leg[2];

        bool blocks(const point<T> &istart, const point<T> &iend) const
        {
            if(istart == *this || iend == *this) throw std::invalid_argument(std::string(__FILE__) + ": Cannot start or end on pin");
            const point<T> start = istart - *this;
            const point<T> end = iend - *this;
            const point<T> leg1 = leg[0] - *this;
            const point<T> leg2 = leg[1] - *this;

            // Check for trival case
            if(leg1.magnitude_squared() == 0
                || leg2.magnitude_squared() == 0
                || leg1 == leg2
            ) return false;

            const T leg1_cross_leg2 = leg1.cross(leg2);
            const T leg2_cross_leg1 = leg2.cross(leg1);

            // Handle the case where the pin is in a line
            if(leg1_cross_leg2 == 0)
            {
                // leg1 and leg2 are parallel
                if(leg1.dot(leg2) > 0) return true;

                // leg1 and leg2 are antiparallel
                assert(leg1.dot(leg2) < 0);
                const bool on_left = leg1.cross(start) > 0;
                if(on_left)
                {
                    const bool also_on_left = leg1.cross(end) > 0;
                    return !also_on_left;
                }
                else
                {
                    const bool also_on_right = leg1.cross(end) < 0;
                    return !also_on_right;
                }
            }

            const bool start_inside = leg1.cross(start) * leg1_cross_leg2 >= 0
                && leg2.cross(start) * leg2_cross_leg1 >= 0;

            const bool end_inside = leg1.cross(end) * leg1_cross_leg2 >= 0
                && leg2.cross(end) * leg2_cross_leg1 >= 0;
            
            return start_inside != end_inside;
        }
    };
}

#endif