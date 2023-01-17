#ifndef GLNAV_OBSTRUCTION_WRAPPER_H
#define GLNAV_OBSTRUCTION_WRAPPER_H

#include "glnav_obstacle_interface.h"
#include "glnav_path.h"
#include <map>

namespace glnav
{
    template<typename T>
    class obstruction_wrapper : virtual public obstacle_interface<T>
    {
    public:
        obstruction_wrapper(const obstacle_interface<T> *source)
            : __source(source)
        { }

        virtual bool obstructs(const path<T> &input) const
        {
            // Check for cache
            typename map_t::const_iterator it = this->__cache.find(input);
            if(it != this->__cache.end())
                return it->second;

            // Not found in cache
            const bool result = this->__source->obstructs(input);
            this->__cache.insert(input, result);
            return result;
        }

        virtual point_group<T> corners() const { return this->__source->corners(); }

        size_t cache_size() const { return this->__cache.size(); }
        void clear_cache() const { this->__cache.clear(); }

    private:
        typedef std::map<path<T> , bool> map_t;
        const obstacle_interface<T> *__source;
        map_t __cache;
    };
}

#endif