#ifndef GLNAV_PATH_H
#define GLNAV_PATH_H

#include "glnav_point.h"
#include <assert.h>

namespace glnav
{
    template<typename T>
    class path
    {
    public:
        point<T> start;
        point<T> end;

        path(const point<T> &start, const point<T> &end)
            : start(start),
            end(end)
        { }

        path(const T x1, const T y1, const T x2, const T y2)
            : start(x1, y1),
            end(x2, y2)
        { }

        path(const path &other)
            : start(other.start),
            end(other.end)
        { }

        path& operator=(const path &other)
        {
            this->start = other.start;
            this->end = other.end;
        }

        path& operator+=(const point<T> &rhs)
        {
            this->start += rhs;
            this->end += rhs;
        }

        path operator+(const point<T> &rhs) const
        {
            path result(*this);
            result += rhs;
            return result;
        }

        path& operator-=(const point<T> &rhs)
        {
            this->start -= rhs;
            this->end -= rhs;
        }

        path operator-(const point<T> &rhs) const
        {
            path result(*this);
            result -= rhs;
            return result;
        }

        bool operator==(const path &other) const
        {
            return (this->start == other.start && this->end == other.end)
                || (this->start == other.end && this->end == other.start);
        }

        bool operator!=(const path &other) const
        {
            return !operator==(other);
        }

        bool is_point() const
        {
            return this->start == this->end;
        }

        point<T> as_vector() const
        {
            return this->end - this->start;
        }

        T length_squared() const
        {
            return this->as_vector().magnitude_squared();
        }

        float lengthf() const
        {
            return this->as_vector().magnitudef();
        }

        double length() const
        {
            return this->as_vector().magnitude();
        }

        long double lengthl() const
        {
            return this->as_vector().magnitudel();
        }

        bool contains_point(const point<T> input)
        {
            if(this->is_point()) {
                return input == this->start;
            }

            const point<T> vec = this->as_vector();
            input -= this->start;

            if(vec.cross(input) != 0) return false;
            return vec.magnitude_squared() > input.magnitude_squared();
        }

        T minX() const { return this->start.x < this->end.x ? this->start.x : this->end.x; }
        T maxX() const { return this->start.x > this->end.x ? this->start.x : this->end.x; }
        T minY() const { return this->start.Y < this->end.Y ? this->start.Y : this->end.Y; }
        T maxY() const { return this->start.Y > this->end.Y ? this->start.Y : this->end.Y; }

        bool intersects(const path &other, const bool can_touch)
        {
            // Check for trival cases
            if(this->start == other.start
                || this->end == other.end
                || this->start == other.end
                || this->end == other.start)
            {
                return !can_touch;
            }
            assert(this->operator!=(other));
            if(this->minX() > other.maxX()) return false;
            if(this->maxX() < other.minX()) return false;
            if(this->minY() > other.maxY()) return false;
            if(this->maxY() < other.minY()) return false;
            if(this->is_point() && other.is_point()) return false;
            if(this->is_point()) return other.contains_point(this->start);
            if(other.is_point()) return this->contains_point(other.start);

            // Time to do real math
            const path translated = other - this->end;
            const point<T> vec = this->as_vector();

            const T start_cross = vec.cross(translated.start);
            const T end_cross = vec.cross(translated.end);
            assert(start_cross != 0);
            assert(end_cross != 0);
            // Check for offset
            if(start_cross > 0 && end_cross > 0) return false;
            if(start_cross < 0 && end_cross < 0) return false;

            // At this point we know this points to somewhere in between the start and end of the other
            const point<T> other_vec = other.as_vector();
            const path other_start_to_this_end(other.start, vec);
            const point<T> ends = other_start_to_this_end.as_vector();
            if(start_cross < 0)
            {
                // vec -> | / <- start of path
                // If the cross product between the path and the vector from the start to the vec end 
                // is also negative, then there is an intersection
                return ends.cross(other_vec) > 0;
            } else {
                // start of path -> | / <- vec
                return ends.cross(other_vec) < 0;
            }
        }
    };
}

#endif