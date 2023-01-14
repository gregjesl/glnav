#ifndef GLNAV_PATH_H
#define GLNAV_PATH_H

#include "glnav_point.h"
#include <assert.h>
#include <stdexcept>

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
            return *this;
        }

        path operator-(const point<T> &rhs) const
        {
            path result(*this);
            result -= rhs;
            return result;
        }

        /*! \brief Determines if the paths are parallel
         * 
         * \note This method will always return true if one of the paths is a point
         */
        bool is_parallel(const path<T> &input) const
        {
            return this->as_vector().cross(input.as_vector()) == 0;
        }

        bool operator==(const path &other) const
        {
            const bool result = (this->start == other.start && this->end == other.end)
                || (this->start == other.end && this->end == other.start);
            #if DEBUG
            if(result) assert(this->is_parallel(other));
            #endif
            return result;
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

        /*! \brief Checks connection to another path 
         *
         * \return True if this path starts or ends on the other path's start or end points, false otherwise
         */
        bool connects_to(const path<T> &other) const
        {
            return this->start == other.start
                || this->end == other.end
                || this->start == other.end
                || this->end == other.start;
        }

        bool contains_point(const point<T> &input, const bool include_endpoints) const
        {
            if(input == this->start || input == this->end) return include_endpoints;
            path<T> tranlsated = *this - input;
            return tranlsated.start.cross(tranlsated.end) == 0
                    && tranlsated.start.dot(tranlsated.end) < 0;
        }

        bool terminates_within(const path<T> &other) const
        {
            return other.contains_point(this->start, false) || other.contains_point(this->end, false);
        }

        bool overlaps(const path<T> &other) const
        {
            if(!this->is_parallel(other)) return false;
            return this->terminates_within(other) || other.terminates_within(*this);
        }

        T minX() const { return this->start.x < this->end.x ? this->start.x : this->end.x; }
        T maxX() const { return this->start.x > this->end.x ? this->start.x : this->end.x; }
        T minY() const { return this->start.y < this->end.y ? this->start.y : this->end.y; }
        T maxY() const { return this->start.y > this->end.y ? this->start.y : this->end.y; }

        /*! \brief Rapidly eliminates paths that do not intersect 
         * 
         * This method attempts to rapidly eliminate the possibility of intersection
         * by seeing if the two paths are even in the same region
         * 
         * \returns False if there is no possibility of the two paths intersecting, otherwise true
         */
        bool could_intersect(const path<T> &other) const
        {
            if(this->minX() > other.maxX()) return false;
            if(this->maxX() < other.minX()) return false;
            if(this->minY() > other.maxY()) return false;
            if(this->maxY() < other.minY()) return false;
            return true;
        }

        bool intersects(const path &other, const bool terminate_without_intersect) const
        {
            // Fast rule-out
            if(!this->could_intersect(other)) return false;

            // Check for equality
            if(this->operator==(other))
            {
                if(this->is_point()) {
                    assert(other.is_point());
                    return !terminate_without_intersect;
                }
                assert(this->overlaps(other));
                return true;
            }
            assert(*this != other);

            // Check for the case where one or both is a point
            {
                const bool point_this = this->is_point();
                const bool point_that = other.is_point();
                
                if(point_this && point_that) 
                { 
                    assert(*this != other); 
                    return false;
                }
                if(point_this)
                {
                    if(terminate_without_intersect) return false;
                    return other.contains_point(this->start, true);
                }
                if(point_that)
                {
                    if(terminate_without_intersect) return false;
                    return this->contains_point(other.start, true);
                }
            }
            assert(!this->is_point() && !other.is_point());

            if(this->connects_to(other))
            {
                if(this->terminates_within(other))
                    return true;
                if(other.terminates_within(*this))
                    return true;
                return !terminate_without_intersect;
            }
            if(this->overlaps(other)) return true;

            // At this point, they must either intersect in the middle of each other
            // or they do not intersect
            const path translated = other - this->end;
            const point<T> vec = this->as_vector();
            const T start_cross = vec.cross(translated.start);
            const T end_cross = vec.cross(translated.end);
            const T start_dot = vec.dot(translated.start);
            const T end_dot = vec.dot(translated.end);

            // The following two cases should have been addressed by ::_regional_overlap()
            assert(start_dot >= 0);
            assert(end_dot >= 0);

            if(start_cross > 0 && end_cross > 0) return false; // Other is to the right of this
            if(start_cross < 0 && end_cross < 0) return false; // Other is to the left of this

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

        bool crosses(const point<T> fence[3])
        {
            const point<T> &bottom = fence[0];
            const point<T> &center = fence[1];
            const point<T> &top = fence[2];
            const path<T> top_path(top, center);
            const path<T> bottom_path(bottom, center);

            // Validate input
            if(top == bottom || top == center || bottom == center) throw std::invalid_argument("Invalid fence");

            // Check for trivial cases
            if(this->start == top || this->start == center || this->start == bottom) return false;
            if(this->end == top || this->end == center || this->end == bottom) return false;

            // Check for changing cross signs
            const point<T> to_top = top - center;
            const point<T> to_bottom = bottom - center;
            const point<T> to_start = this->start - center;
            const point<T> to_end = this->end - center;
            const T cross_top_start = to_top.cross(to_start);
            const T cross_top_end = to_top.cross(to_end);
            const T cross_bottom_start = to_bottom.cross(to_start);
            const T cross_bottom_end = to_bottom.cross(to_end);

            // 3^4 cases to consider

            // Handle the case where the path skims the fence
            if(cross_top_start == 0
                || cross_top_end == 0
                || cross_bottom_start == 0 
                || cross_top_end == 0
            ) return false;
            
            // Now 2^4 cases to handle

            // Handle the case where start and end are on one side of the fence
            // Use the dot product

        }
    };
}

#endif