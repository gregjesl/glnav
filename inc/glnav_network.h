#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_obstacle_interface.h"
#include <assert.h>
#include <map>

namespace glnav
{
    template<typename T>
    class edge
    {
    public:
        const fixed_point<T> * lower;
        const fixed_point<T> * upper;

        edge(const fixed_point<T> * node1, const fixed_point<T> * node2)
            : lower(node1 < node2 ? node1 : node2),
            upper(node1 > node2 ? node1 : node2)
        {
            assert(node1 != NULL);
            assert(node2 != NULL);
            assert(node1 != node2);
            assert(*node1 != *node2);
        }

        edge(const edge &other)
            : lower(other.lower),
            upper(other.upper)
        {
            assert(this->lower != NULL);
            assert(this->upper != NULL);
            assert(this->lower < this->upper);
        }

        edge& operator= (const edge &other)
        {
            this->lower = other.lower;
            this->upper = other.upper;
            assert(this->lower != NULL);
            assert(this->upper != NULL);
            assert(this->lower < this->upper);
        }

        path<T> as_path() const
        {
            return path<T>(
                this->lower->x(),
                this->lower->y(),
                this->upper->x(),
                this->upper->y()
            );
        }

        bool operator== (const edge<T> &other) const
        {
            assert(this->lower != NULL);
            assert(this->upper != NULL);
            assert(other.lower != NULL);
            assert(other.upper != NULL);
            assert(this->lower < this->upper && other.lower < other.upper);
            return this->lower == other.lower && this->upper == other.upper;
        }

        bool operator!= (const edge<T> &other) const
        {
            assert(this->lower != NULL);
            assert(this->upper != NULL);
            assert(other.lower != NULL);
            assert(other.upper != NULL);
            assert(this->lower < this->upper && other.lower < other.upper);
            return this->lower != other.lower || this->upper != other.upper;
        }

        bool operator< (const edge &other) const
        {
            assert(this->lower != NULL);
            assert(this->upper != NULL);
            assert(other.lower != NULL);
            assert(other.upper != NULL);
            assert(this->lower < this->upper && other.lower < other.upper);
            if(this->lower < other.lower) return true;
            if(this->lower > other.lower) return false;
            assert(this->lower == other.lower);
            if(this->upper < other.upper) return true;
            if(this->upper > other.upper) return false;
            assert(*this == other);
            return false;
        }
    };

    template<typename T>
    class network
    {
    public:
        network(const obstacle_group<T> * obstacles)
            : __obstacles(obstacles)
        {
            if(obstacles == NULL) throw std::invalid_argument("Null pointer");

            this->__nodes = obstacles->corners();

            for(typename point_group<T>::const_iterator it1 = this->__nodes.begin();
                it1 != this->__nodes.end();
                ++it1)
            {
                const fixed_point<T> * point1 = &(*it1);

                // TODO: Optimize
                for(typename point_group<T>::const_iterator it2 = it1;
                    it2 != this->__nodes.end();
                    ++it2)
                {
                    const fixed_point<T> * point2 = &(*it2);

                    // Check for same point
                    if(point1 == point2) continue;

                    // Check for existing edge
                    edge<T> test(point1, point2);
                    if(this->__edges.find(test) != this->__edges.end()) 
                    {
                        continue;
                    }
                    

                    const path<T> test_path = test.as_path();

                    // Check for path
                    if(!obstacles->obstructs(test_path))
                    {
                        this->__edges.insert(
                            std::pair<edge<T> , double>(
                                test, 
                                test_path.length()
                            )
                        );
                    }
                }
            }
        }

        size_t num_nodes() const { return this->__nodes.size(); }
        size_t num_edges() const { return this->__edges.size(); }

        /*
        void navigate(const point<T> &from, const point<T> &to)
        {
            // Check for trivial case
            if(this->__corners.size() == 0)
            {
                // TODO: Return path
                return;
            }

            // typedef std::map<point<T> *>, double> cost_map_t;
            // cost_map_t cost_map;

            // Seed the cost map
        }
        */
    private:
        const obstacle_group<T> *__obstacles;

        point_group<T> __nodes;

        typedef std::map<const edge<T> , double> edge_map_t;
        edge_map_t __edges;
    };
}

#endif