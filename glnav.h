#ifndef GLNAV_H
#define GLNAV_H

#include "inc/glnav_point.h"
#include "inc/glnav_path.h"

#include <vector>
#include <list>
#include <stdexcept>
#include <assert.h>
#include <math.h>

namespace glnav
{

    template<typename T>
    class obstacle
    {
    public:
        obstacle(const std::vector<point<T> > outline)
            : __outline(outline),
            __bounds(compute_bounds(outline))
        { 
            // #error Assert that the outline does not cross itself or contain points
        }

        obstacle(const obstacle& other)
            : __outline(other.outline),
            __bounds(other.bounds)
        {
            assert(this->__bounds == compute_bounds(this->__outline));
        }

        /*
        bool intersects(const path<T> input)
        {
            // #error Iterate through all fences
        }
        */

        const std::vector<point<T> >& outline() const { return this->__outline; }

        const rectangle<T>& bounds() const { return this->__bounds; }

    private:
        static rectangle<T> compute_bounds(std::vector<point<T> > outline)
        {
            if(outline.size() < 3) throw std::invalid_argument("Invalid outline");
            T minX = outline.at(0).X;
            T minY = outline.at(0).Y;
            T maxX = outline.at(0).X;
            T maxY = outline.at(0).Y;

            for(size_t i = 1; i < outline.size(); i++)
            {
                const point<T>& pt = outline.at(i);
                if(pt.X < minX) minX = pt.X;
                if(pt.Y < minY) minY = pt.Y;
                if(pt.X > maxX) maxX = pt.X;
                if(pt.Y > maxY) maxY = pt.Y;
            }
            return rectangle<T>(minX, minY, maxX, maxY);
        }
        std::vector<point<T> > __outline;
        rectangle<T> __bounds;
    };

    template<typename T>
    class route : public std::list<point<T> >
    {

    };
}

#endif