#ifndef GLNAV_OBSTACLE_H
#define GLNAV_OBSTACLE_H

#include "glnav_fence.h"

namespace glnav
{
    /*
    template<typename T>
    class obstacle : public fence<T>
    {
    public:
        obstacle(const std::vector<point<T> > &seed)
            : fence<T>(seed.at(0), seed.at(1))
        {
            if(seed.size() < 3) throw std::invalid_argument("Obstacle must have three points");
            for(size_t i = 2; i < seed.size(); i++)
            {
                this->push_back(seed.at(i));
            }
        }

        virtual ~obstacle()
        { }

        virtual bool obstructs(const path<T> &input) const
        {
            if(fence<T>::obstructs(input)) return true;
            const path<T> segment(*this->back(), *this->front());
            if(input.intersects(segment, true)) return true;
            
            {
                const pin<T> test(*this->back(), this->at(this->size() - 2), this->front());
                if(input.contains_point(test, false))
                {
                    if(test.blocks(input.start, input.end)) return true;
                }
            }

            {
                const pin<T> test(*this->front(), this->back(), this->at(1));
                if(input.contains_point(test, false))
                {
                    if(test.blocks(input.start, input.end)) return true;
                }
            }
        }
    };
    */
}

#endif