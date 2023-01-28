#ifndef GLNAV_POINT_H
#define GLNAV_POINT_H

#include <math.h>
#include <vector>
#include <map>
#include <stdexcept>
#include <assert.h>

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
            if(this->operator==(other)) return false;
            const T mag_this = this->magnitude_squared();
            const T mag_other = other.magnitude_squared();
            if(mag_this < mag_other) return true;
            if(mag_this > mag_other) return false;
            if(this->x > other.x) return true;
            if(this->x < other.x) return false;
            return this->y > other.y;
        }

        bool operator>(const point &other) const
        {
            const T mag_this = this->magnitude_squared();
            const T mag_other = other.magnitude_squared();
            if(mag_this > mag_other) return true;
            if(mag_this < mag_other) return false;
            if(this->x < other.x) return true;
            if(this->x > other.x) return false;
            assert(this->y != other.y);
            return this->y < other.y;
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

        template<typename Q> 
        Q magnitude() const;

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

    template<typename T, typename Q>
    Q angle(const point<T> &input)
    {
        return atan2((Q)input.y, (Q)input.x);
    }

    template<typename T, typename Q>
    Q magnitude(const point<T> &input)
    {
        return sqrt((Q)input.magnitude_squared());
    }

    template<typename T> 
    template<typename Q>
    Q point<T>::magnitude() const
    {
        return glnav::magnitude<T, Q>(*this);
    }

    template<typename T>
    class point_group : public std::vector<point<T> >
    {
    public:
        point_group() : std::vector<point<T> >() { }
        point_group(const std::vector<point<T> > &seed)
            : std::vector<point<T> >(seed)
        { }

        point_group& operator=(const std::vector<point<T> > &seed)
        {
            std::vector<point<T> >::operator=(seed);
            return *this;
        }

        bool contains(const point<T> &key)  const
        {
            for(size_t i = 0; i < this->size(); i++)
            {
                if(this->at(i) == key) return true;
            }
            return false;
        }

        void remove(const point<T> &key)
        {
            size_t i = 0;
            while(i < this->size())
            {
                if(this->at(i) == key) {
                    this->erase(this->begin() + i);
                } else {
                    i++;
                }
            }
        }

        void merge(const point_group<T> &other)
        {
            this->reserve(this->size() + other.size());
            this->insert(this->end(), other.begin(), other.end());
        }
    };

    template<typename T>
    class unknown_point_exception : public std::out_of_range
    {
    public:
        unknown_point_exception(const point<T> &point)
            : std::out_of_range("Point not found"),
            unknown_point(point)
        { }

        point<T> unknown_point;
    };

    template<typename T, typename Q>
    class point_map : public std::map<point<T>, Q>
    {
    public:
        point_map() : std::map<point<T>, Q>() { }
        point_map(const std::map<point<T>, Q> &other)
            : std::map<point<T>, Q>(other)
        { }

        point_map& operator=(const point_map &other)
        {
            std::map<point<T>, Q>::operator=(other);
            return *this;
        }

        bool contains(const point<T> &input) const { return this->find(input) != this->end(); }

        const Q * get(const point<T> &key) const
        {
            typename std::map<point<T>, Q>::const_iterator it = this->find(key);
            return it != this->end() ? &it->second : nullptr;
        }

        const Q force_get(const point<T> &key) const
        {
            typename std::map<point<T>, Q>::const_iterator it = this->find(key);
            if(it == this->end()) throw unknown_point_exception<T>(key);
            return it->second;
        }

        Q * edit(const point<T> &key)
        {
            typename std::map<point<T>, Q>::iterator it = this->find(key);
            return it != this->end() ? &it->second : nullptr;
        }

        bool set(const point<T> &key, const Q value)
        {
            typename std::map<point<T>, Q>::iterator it = this->find(key);
            if(it != this->end()) {
                it->second = value;
            } else {
                this->insert(std::pair<point<T>, Q>(key, value));
            }
            return it == this->end(); // Return true if new
        }

        point_group<T> overlap(const point_map<T, Q> &other) const
        {
            point_group<T> result;
            typename point_map<T, Q>::const_iterator core;
            for(core = this->begin(); core != this->end(); ++core)
            {
                if(other.contains(core->first)) {
                    result.push_back(core->first);
                }
            }
            return result;
        }
    };
}

#endif