#ifndef GLNAV_PATH_H
#define GLNAV_PATH_H

#include "glnav_point.h"
#include "glnav_cartesian.h"
#include <assert.h>
#include <stdexcept>
#include <list>
#include <set>

namespace glnav
{
    template<typename T>
    class path : public cartesian_object<T>
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

        virtual ~path() { }

        static std::vector<path<T> > generate(const std::set<point<T> > &input)
        {
            std::vector<path<T> > result;
            const size_t n = input.size();
            const size_t permutations = (n * (n - 1)) / 2;
            result.reserve(permutations); // https://byjus.com/question-answer/what-is-the-sum-of-1-2-3-n/
            typename std::set<point<T> >::const_iterator lower, upper;
            for(lower = input.begin(); lower != input.end(); ++lower)
            {
                for(upper = lower; upper != input.end(); ++upper)
                {
                    if(*lower == *upper) continue;
                    result.push_back(path<T>(*lower, *upper));
                }
            }
            assert(result.size() == permutations);
            return result;
        }

        path& operator=(const path &other)
        {
            this->start = other.start;
            this->end = other.end;
            return *this;
        }

        path& operator+=(const point<T> &rhs)
        {
            this->start += rhs;
            this->end += rhs;
            return *this;
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

        std::list<point<T> > operator/(const size_t segments)
        {
            if(segments == 0) throw std::invalid_argument("Must have at least one segment");
            std::list<path<T> > result;
            result.push_back(this->start);
            if(segments == 1)
            {
                result.push_back(this->end);
            }
            else if(this->is_point())
            {
                // Do nothing
            }
            else
            {
                const T x_delta = (this->end.x - this->start.x) / segments;
                const T y_delta = (this->end.y - this->start.y) / segments;
                point<T> delta(x_delta, y_delta);
                point<T> next = this->start + delta;
                for(size_t i = 1; i < segments - 1; i++)
                {
                    result.push_back(next);
                    next += delta;
                }
                result.push_back(this->end);
            }
            return result;
        }

        /*
        std::list<path<T> > subdivide_by_count(const size_t segments)
        {
            #error todo
        }

        std::list<path<T> > subdivide_by_length(const double max_length)
        {
            std::list<path<T> > result;
            const double length = this->length();
            if(length < max_length)
            {
                result.push_back(*this);
                return result;
            }
            const size_t segments = (size_t)(this->length() / max_length) + 1;
            #error finish
        }
        */

        /*! \brief Determines if the paths are parallel
         * 
         * \note This method will always return true if one of the paths is a point
         */
        bool is_parallel(const path<T> &input) const
        {
            return this->as_vector().cross(input.as_vector()) == 0;
        }

        point<T> center() const
        {
            return point<T>((this->start.x + this->end.x) / 2, (this->start.y + this->end.y) / 2);
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

        bool operator<(const path &other) const
        {
            // Check for same
            if(*this == other) return false;

            // Prefer shorter
            const T current = this->length_squared();
            const T other_mag = other.length_squared();
            if(current < other_mag) return true;
            if(current > other_mag) return false;
            assert(current == other_mag);

            // Prefer the one centered closer
            const point<T> current_center = this->center();
            const point<T> other_center = other.center();
            if(current_center < other_center) return true;
            if(current_center > other_center) return false;
            assert(current_center == other_center);
            assert(!this->is_point() && !other.is_point());

            // Use the starting point
            return this->start < other.start;
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

        template<typename Q>
        Q length() const;

        virtual double cost() const
        {
            return this->length<double>();
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
            if(!this->could_contain(input, include_endpoints)) return false;
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
            return this->could_overlap(other, true);
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
                assert(this->overlaps(other) || *this == other);
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

            // Check for starting or ending on each other
            if(this->contains_point(other.start, false)) return !terminate_without_intersect;
            if(this->contains_point(other.end, false)) return !terminate_without_intersect;
            if(other.contains_point(this->start, false)) return !terminate_without_intersect;
            if(other.contains_point(this->end, false)) return !terminate_without_intersect;

            // At this point, they must either intersect in the middle of each other
            // or they do not intersect
            {
                const path translated = other - this->end;
                const point<T> vec = this->as_vector();
                const T start_cross = vec.cross(translated.start);
                const T end_cross = vec.cross(translated.end);

                if(start_cross > 0 && end_cross > 0) return false; // Other is to the right of this
                if(start_cross < 0 && end_cross < 0) return false; // Other is to the left of this
            }
            
            {
                const path translated = *this - other.end;
                const point<T> vec = other.as_vector();
                const T start_cross = vec.cross(translated.start);
                const T end_cross = vec.cross(translated.end);

                if(start_cross > 0 && end_cross > 0) return false; // This is to the right of other
                if(start_cross < 0 && end_cross < 0) return false; // This is to the left of other
            }

            return true;
        }
    };

    template<typename T>
    template<typename Q>
    Q path<T>::length() const
    {
        return sqrt((Q)this->as_vector().magnitude_squared());
    }

    template<typename T, typename Q>
    Q length(const path<T> &input)
    {
        return sqrt((Q)input.as_vector().magnitude_squared());
    }
}

#endif