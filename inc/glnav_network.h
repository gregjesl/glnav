#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include "glnav_obstacle_interface.h"
#include <assert.h>
#include <map>

namespace glnav
{
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
                const point<T> * point1 = &(*it1);

                // TODO: Optimize
                for(typename point_group<T>::const_iterator it2 = it1;
                    it2 != this->__nodes.end();
                    ++it2)
                {
                    const point<T> * point2 = &(*it2);

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