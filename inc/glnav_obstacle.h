#ifndef GLNAV_OBSTACLE_H
#define GLNAV_OBSTACLE_H

#include "glnav_cartesian.h"
#include "glnav_obstacle_interface.h"
#include "glnav_point.h"
#include <vector>
#include <string>

namespace glnav
{
    template<typename T>
    class obstacle : virtual public obstacle_interface<T>
    {
    public:
        obstacle(const std::vector<point<T> > &outline)
            : __corners(outline),
            __minX(outline.front().x),
            __minY(outline.front().y),
            __maxX(outline.front().x),
            __maxY(outline.front().y)
        {
            if(outline.size() < 3) throw std::invalid_argument("Insufficent points");
            if(point_group<T>(outline).size() < outline.size()) throw std::invalid_argument("Duplicate points");

            float angle = 0;
            for(size_t i = 1; i < outline.size(); i++)
            {
                const point<T> vec = outline.at(i) - outline.at(i-1);
                assert(vec.magnitude_squared() > 0);
                angle += glnav::angle<T, float>(vec);
            }
            const point<T> vec = outline.back() - outline.front();
            angle += glnav::angle<T, float>(vec);
            assert(angle > 0.98f * M_PI * 2 || angle < 0.98f * M_PI * 2);
            assert(fabs(angle) < 1.02f * M_PI * 2);
            this->__clockwise = angle > 0;

            for(size_t i = 1; i < outline.size(); i++)
            {
                this->__update_axis(outline.at(i).x, this->__minX, this->__maxX);
                this->__update_axis(outline.at(i).y, this->__minY, this->__maxY);
            }
        }

        obstacle(const obstacle &other)
            : __corners(other.__corners),
            __clockwise(other.__clockwise)
        { }

        obstacle& operator=(const obstacle &other)
        {
            this->__corners = other.__corners;
            this->__clockwise = other.__clockwise;
            return this;
        }

        static obstacle square(const point<T> &center, const T side_length)
        {
            const T half_side = side_length / 2;
            std::vector<point<T> > outline;
            outline.push_back(center + point<T>(half_side, half_side));
            outline.push_back(center + point<T>(-half_side, half_side));
            outline.push_back(center + point<T>(-half_side, -half_side));
            outline.push_back(center + point<T>(half_side, -half_side));
            return obstacle(outline);
        }

        static obstacle regular_polygon(const point<T> &center, const T radius, const double rotation, const size_t num_points)
        {
            if(num_points < 3) throw std::invalid_argument("Polygon must have at least three sides");
            std::vector<point<T> > outline;
            for(size_t i = 0; i < num_points; i++)
            {
                const double ratio = ((double)i) / ((double)num_points);
                const double angle = M_PI * 2 * ratio + rotation;
                outline.push_back(center + point<T>(radius * cos(angle), radius * sin(angle)));
            }
            return obstacle(outline);
        }

        virtual bool obstructs(const path<T> &input) const
        {
            // Fast exclude
            if(!this->could_overlap(input, true)) return false;

            assert(this->__corners.size() > 2);
            if(this->__corners_obstruct(input)) return true;
            if(this->__sides_obstruct(input)) return true;
            return false;
        }

        virtual point_group<T> corners() const
        {
            return point_group<T>(this->__corners);
        }

        point_group<T> corners(const point<T> &from) const
        {
            point_group<T> result;
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                const path<T> test(from, this->__corners.at(i));
                if(!this->obstructs(test)) {
                    result.push_back(this->__corners.at(i));
                }
            }
            return result;
        }

        virtual T minX() const { return this->__minX; }
        virtual T maxX() const { return this->__maxX; }
        virtual T minY() const { return this->__minY; }
        virtual T maxY() const { return this->__maxY; }

        std::string svg() const
        {
            std::string result = "<polygon points=\"";
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                result += std::to_string(this->__corners.at(i).x);
                result.push_back(',');
                result += std::to_string(this->__corners.at(i).y);
                result.push_back(' ');
            }
            result += "\"/>\n";
            return result;
        }

    private:
        std::vector<point<T> > __corners;
        bool __clockwise;
        T __minX;
        T __minY;
        T __maxX;
        T __maxY;

        bool __corner_obstructs(const path<T> &input, const size_t index) const
        {
            assert(index < this->__corners.size());
            const point<T> &center = this->__corners.at(index);
            const point<T> &leg1 = index > 0 ? this->__corners.at(index - 1) : this->__corners.back();
            const point<T> &leg2 = index < this->__corners.size() - 1 ? this->__corners.at(index + 1) : this->__corners.front();
            const corner<T> test(center, leg1, leg2, this->__clockwise);
            return test.obstructs(input);
        }

        bool __corners_obstruct(const path<T> &input) const
        {
            assert(this->__corners.size() > 2);
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                if(this->__corner_obstructs(input, i)) return true;
            }
            return false;
        }

        bool __side_obstructs(const path<T> &input, const size_t index) const
        {
            assert(index < this->__corners.size());
            const path<T> wall(
                index == 0 ? this->__corners.back() : this->__corners.at(index - 1),
                this->__corners.at(index)
            );
            if(input == wall || input.overlaps(wall)) return false;
            if(input.terminates_within(wall)) return true;
            return wall.intersects(input, true);
        }

        bool __sides_obstruct(const path<T> &input) const
        {
            assert(this->__corners.size() > 2);
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                if(this->__side_obstructs(input, i)) return true;
            }
            return false;
        }

        void __update_min(const T input, T &value)
        {
            if(input < value) value = input;
        }

        void __update_max(const T input, T &value)
        {
            if(input > value) value = input;
        }

        void __update_axis(const T value, T &min, T &max)
        {
            this->__update_min(value, min);
            this->__update_max(value, max);
        }
    };
}

#endif