#ifndef GLNAV_OBSTACLE_H
#define GLNAV_OBSTACLE_H

#include "glnav_cartesian.h"
#include "glnav_obstacle_interface.h"
#include "glnav_point.h"

namespace glnav
{
    template<typename T>
    class obstacle : public obstacle_interface<T>
    {
    public:
        obstacle(const std::vector<point<T> > &outline)
            : __corners(outline)
        {
            if(outline.size() < 3) throw std::invalid_argument("Insufficent points");
            if(point_group<T>(outline).size() < outline.size()) throw std::invalid_argument("Duplicate points");

            float angle = 0;
            for(size_t i = 1; i < outline.size(); i++)
            {
                const point<T> vec = outline.at(i) - outline.at(i-1);
                assert(vec.magnitude_squared() > 0);
                angle += vec.anglef();
            }
            const point<T> vec = outline.back() - outline.front();
            angle += vec.anglef();
            assert(angle > 0.98f * M_2_PI || angle < 0.98f * M_2_PI);
            assert(fabs(angle) < 1.02f * M_2_PI);
            this->__clockwise = angle > 0;
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

        virtual bool obstructs(const path<T> &input) const
        {
            // Start with a fast exclude
            // TODO: Fast exclude

            assert(this->__corners.size() > 2);
            if(this->__corners_obstruct(input)) return true;
            if(this->__sides_obstruct(input)) return true;
            return false;
        }

        virtual point_group<T> corners() const
        {
            return point_group<T>(this->__corners);
        }

    private:
        std::vector<point<T> > __corners;
        bool __clockwise;

        bool __corner_obstructs(const path<T> &input, const size_t index)
        {
            assert(index < this->__corners.size());
            const point<T> &center = this->__corners.at(index);
            const point<T> &leg1 = index > 0 ? this->__corners.at(index - 1) : this->__corners.back();
            const point<T> &leg2 = index < this->__corners.size() - 1 ? this->__corners.at(index + 1) : this->__corners.front();
            const corner<T> test(center, leg1, leg2, this->__clockwise);
            return test.obstructs(input);
        }

        bool __corners_obstruct(const path<T> &input)
        {
            assert(this->__corners.size() > 2);
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                if(this->__corner_obstructs(input, index)) return true;
            }
            return false;
        }

        bool __side_obstructs(const path<T> &input, const size_t index)
        {
            assert(index < this->__corners.size());
            const path<T> wall(
                index == 0 ? this->__corners.back() : this->__corners.at(index - 1),
                this->__corners.at(index)
            );
            return wall.intersects(input, true);
        }

        bool __sides_obstruct(const path<T> &input)
        {
            assert(this->__corners.size() > 2);
            for(size_t i = 0; i < this->__corners.size(); i++)
            {
                if(this->__side_obstructs(input, i)) return true;
            }
            return false;
        }
    };
}

#endif