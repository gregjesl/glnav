#ifndef GLNAV_DIJKSTRA_H
#define GLNAV_DIJKSTRA_H

#include "glnav_version_control.h"
#include "glnav_maze.h"
#include "glnav_cost_map.h"
#include "glnav_route.h"
#include <stdexcept>
#include <set>
#include <stack>

namespace glnav
{
    template<typename T, typename Q>
    class dijkstra : public version_dependent
    {
    public:
        dijkstra(const maze<T, Q> &input)
            : version_dependent(input), 
            __maze(input),
            __map(input.seed_map())
        { 
            this->__to_go.insert(input.finish());
            this->__map.set(this->__maze.finish(), 0);
        }

        virtual ~dijkstra()
        { }

        void iterate()
        {
            // Check for version mismatch
            if(!this->versions_synchronized(this->__maze)) 
                throw version_mismatch(*this, this->__maze);

            // Check for solution
            if(this->is_solved()) return;
            assert(!this->__to_go.empty());

            // Pop the front
            const point<T> next(*this->__to_go.begin());
            this->__to_go.erase(next);

            // Get the neighbors
            const neighborhood<T, Q> neighbors(this->__maze.neighbors(next));

            // Get the cost to go
            const Q cost_to_go = this->__map.cost(next);

            // Iterate through the neighbors
            for(size_t i = 0; i < neighbors.size(); i++)
            {
                const point<T> & location = neighbors.at(i).location();
                assert(this->__map.contains(location));
                const Q test_cost = cost_to_go + neighbors.at(i).cost;
                if(test_cost < this->__map.cost(location))
                {
                    this->__map.set(location, test_cost);
                    if(this->__to_go.find(location) == this->__to_go.end())
                        this->__to_go.insert(location);
                }
            }
        }

        bool is_solved() const { return this->__to_go.empty(); }

        route<T, Q> build_route() const
        {
            route<T, Q> result(this->__maze.start());
            if(!this->is_solved()) return result;

            point<T> current = this->__maze.start();

            while(current != this->__maze.finish())
            {
                const neighborhood<T, Q> neighbors = this->__maze.neighbors(current);
                assert(neighbors.size() > 0);
                point<T> next = neighbors.at(0).location();
                double cost = this->__map.cost(next);
                for(size_t i = 1; i < neighbors.size(); i++)
                {
                    if(this->__map.cost(neighbors.at(i).location()) < cost) {
                        next = neighbors.at(i).location();
                        cost = this->__map.cost(next);
                    }
                }

                // Check for loop
                for(size_t i = 0; i < result.size(); i++)
                {
                    if(result[i].target == next) throw std::runtime_error("Loop detected");
                }

                // Set the speed
                const Q delta_cost = this->__map.cost(current) - this->__map.cost(next);
                const path<T> next_path = path<T>(current, next);
                const Q distance = glnav::length<T, Q>(next_path);

                // Set the new waypoint
                result.push_back(heading<T, Q>(next, distance / delta_cost));
                
                // Move the points
                current = next;
            }

            return result;
        }

        void reset()
        {
            this->__map = this->__maze.seed_map();
            this->__map.set(this->__maze.finish(), 0);
            this->__to_go.clear();
            this->__to_go.insert(this->__maze.finish());
            this->synchronize_version(this->__maze);
        }
    private:
        const maze<T, Q> & __maze;
        cost_map<T, Q> __map;
        std::set<point<T> > __to_go;
    };
}

#endif