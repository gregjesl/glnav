#ifndef GLNAV_POINT_H
#define GLNAV_POINT_H

#include <math.h>

namespace glnav
{
    template<typename T>
    class point
    {
    public:
        point() : x(0), y(0) { }
        point(const T xseed, const T yseed) : x(xseed), y(yseed) { }
        point(const point &other) : x(other.x), y(other.y) { }
        point& operator=(const point &other) { this->x = other.x; this->y = other.y; }
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
    };
}

#endif