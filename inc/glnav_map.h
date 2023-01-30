#ifndef GLNAV_MAP_H
#define GLNAV_MAP_H

#include "con3/con3.h"
#include "glnav_network.h"
#include "glnav_path.h"
#include "glnav_obstacle.h"

namespace glnav
{
    template<typename T>
    class map : public con3::set<obstacle<T> *>
    {
    public:
        map()
            : con3::set<obstacle<T> *>()
        { }

        bool obstructs(const path<T> &input)
        {
            for(size_t i = 0; i < this->values().size(); i++)
            {
                if(this->at(i)->obstructs(input)) return true;
            }
            return false;
        }

        std::set<point<T> > corners()
        {
            std::set<point<T> > result;

            // Iterate through the obstacles
            for(size_t i = 0; i < this->values().size(); i++)
            {
                const point_group<T> corners = this->at(i)->corners();

                // Iterate through the corners
                for(size_t j = 0; j < corners.size(); j++)
                {
                    result.insert(corners.at(j));
                }
            }
            return result;
        }
    };
}

#endif