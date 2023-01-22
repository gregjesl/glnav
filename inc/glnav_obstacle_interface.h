#ifndef GLNAV_OBSTACLE_INTERFACE_H
#define GLNAV_OBSTACLE_INTERFACE_H

#include "glnav_point.h"
#include "glnav_path.h"
#include "glnav_cartesian.h"
#include <set>

namespace glnav
{
    template<typename T>
    class obstacle_interface : virtual public cartesian_object<T>
    {
    public:
        virtual bool obstructs(const path<T> &input) const = 0;
        virtual point_group<T> corners() const = 0;
    };

    template<typename T>
    class obstacle_group
    {
    public:
        obstacle_group()
            : __obstacles()
        { }

        obstacle_group(const obstacle_group &other)
            : __obstacles(other.__obstacles)
        { }

        obstacle_group& operator=(const obstacle_group &other)
        {
            this->__obstacles.clear();
            this->__obstacles = other.__obstacles;
        }

        virtual ~obstacle_group() { }

        void add(const obstacle_interface<T> * input)
        {
            this->__obstacles.insert(input);
        }

        void remove(const obstacle_interface<T> * input)
        {
            this->__obstacles.erase(input);
        }

        obstacle_group filter(const T x1, const T y1, const T x2, const T y2)
        {
            cartesian_area<T> area(x1, y1, x2, y2);
            obstacle_group result;
            for(typename obstacle_ptr_set_t::const_interator it = this->__obstacles.begin(); 
                it != this->__obstacles.end(); 
                ++it)
            {
                if(area.could_overlap(**it, true)) {
                    result.insert(*it);
                }
            }
            return result;
        }

        virtual bool obstructs(const path<T> &input) const
        {
            for(typename obstacle_ptr_set_t::const_iterator it = this->__obstacles.begin(); 
                it != this->__obstacles.end(); 
                ++it)
            {
                const obstacle_interface<T> * obs = *it;
                if(obs->obstructs(input)) return true;
            }
            return false;
        }

        virtual point_group<T> corners() const
        {
            point_group<T> result;
            for(typename obstacle_ptr_set_t::const_iterator it = this->__obstacles.begin(); 
                it != this->__obstacles.end(); 
                ++it)
            {
                const obstacle_interface<T> * obs = *it;
                result.merge(obs->corners());
            }
            return result;
        }

        bool contains(const point<T> &input) const
        {
            return this->corners().contains(input);
        }

        size_t size() const { return this->__obstacles.size(); }

    private:
        typedef std::set<const obstacle_interface<T> *> obstacle_ptr_set_t;
        obstacle_ptr_set_t __obstacles;
    };
}

#endif