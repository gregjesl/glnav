#ifndef GLNAV_LEVEL_H
#define GLNAV_LEVEL_H

#include "glnav_point.h"
#include "glnav_obstacle_interface.h"
#include "glnav_obstruction_wrapper.h"
#include <set>

namespace glnav
{
    template<typename T>
    class level
    {
    public:
        void add(const obstacle_interface<T> *input)
        {
            this->__obstacles.insert(input);
        }

        void remove(const obstacle_interface<T> *input)
        {
            this->__obstacles.erase(input);
        }

        void navigate(const point<T> &from, const point<T> &to)
        {
            (void)from;
            (void)to;
            // Step 1: Find all potential obstacles

            // Step 2: Build the network

            // Step 3: Translate the network such that the goal is to navigate to (0,0)

            // Step 4: A*
        }
    private:
        typedef std::set<obstacle_interface *> obstacle_set_t;
        obstacle_set_t __obstacles;

        obstacle_set_t __load(const T x1, const T y1, const T x2, const T y2)
        {
            cartesian_area<T> area(x1, y1, x2, y2);
            obstacle_set_t result;
            for(obstacle_set_t::const_interator it = this->__obstacles.begin(); it != this->__obstacles.end(); ++it)
            {
                if(area.could_overlap(**it)) {
                    result.insert(*it);
                }
            }
            return result;
        }
    };
}

#endif