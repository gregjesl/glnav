#ifndef GLNAV_POINT_H
#define GLNAV_POINT_H

#include <math.h>
#include <stdexcept>

namespace glnav
{
    template<typename T>
    class point
    {
    public:
        point() : x(0), y(0) { }
        point(const T xseed, const T yseed) : x(xseed), y(yseed) { }
        point(const point &other) : x(other.x), y(other.y) { }
        point& operator=(const point &other) { this->x = other.x; this->y = other.y; return *this; }
        T x;
        T y;
        point& operator+=(const point &other)
        {
            this->x += other.x;
            this->y += other.y;
            return *this;
        }
        point operator+(const point &other) const
        {
            point result(*this);
            result += other;
            return result;
        }
        point& operator-=(const point &other)
        {
            this->x -= other.x;
            this->y -= other.y;
            return *this;
        }
        point operator-(const point &other) const
        {
            point result(*this);
            result -= other;
            return result;
        }
        bool operator==(const point &other) const
        {
            return this->x == other.x && this->y == other.y;
        }

        bool operator!=(const point &other) const
        {
            return this->x != other.x || this->y != other.y;
        }

        bool operator<(const point &other) const
        {
            if(this->magnitude_squared() < other.magnitude_squared()) return true;
            return this->x > other.x;
        }

        bool operator>(const point &other) const
        {
            if(this->magnitude_squared() > other.magnitude_squared()) return true;
            return this->x > other.x;
        }
        
        T dot(const point &other) const
        {
            return (this->x * other.x) + (this->y * other.y);
        }

        T cross(const point &other) const
        {
            return (this->x * other.y) - (this->y * other.x);
        }

        T magnitude_squared() const
        {
            return this->dot(*this);
        }

        float magnitudef() const
        {
            return sqrtf((float)this->magnitude_squared());
        }

        double magnitude() const
        {
            return sqrt((double)this->magnitude_squared());
        }

        long double magnitudel() const
        {
            return sqrtl((long double)this->magnitude_squared());
        }

        float anglef() const
        {
            return atan2f(this->y, this->x);
        }

        double angle() const
        {
            return atan2(this->y, this->x);
        }

        long double anglel() const
        {
            return atan2l(this->y, this->x);
        }

        /*! \brief Determiens if a point lies within the concave section defined by two legs
         *
         * From https://stackoverflow.com/questions/13640931/how-to-determine-if-a-vector-is-between-two-other-vectors
         * 
         * \warning Will return `true` if this is the same as the reference
         */
        bool is_between(const point<T> &reference, point<T> leg1, point<T> leg2) const
        {
            const point<T> test = *this - reference;
            leg1 -= reference;
            leg2 -= reference;
            (void)reference;

            const T leg1_cross_leg2 = leg1.cross(leg2);
            const T leg2_cross_leg1 = leg2.cross(leg1);

            // Handle the case where the pin is in a line
            if(leg1_cross_leg2 == 0) throw std::invalid_argument("Legs are parallel");

            return leg1.cross(test) * leg1_cross_leg2 >= 0
                && leg2.cross(test) * leg2_cross_leg1 >= 0;
        }

        enum defined_rotation
        {
            eighth_clockwise,
            eighth_counterclockwise,
            quarter_clockwise,
            quarter_counterclockwise
        };

        point<T>& rotate(const point<T> &around, const enum defined_rotation)
        {
            (void)around;
        }
    };
}

#endif