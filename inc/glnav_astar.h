#ifndef GLNAV_ASTAR_H
#define GLNAV_ASTAR_H

#include "glnav_maze.h"
#include "glnav_route.h"
#include <algorithm>

namespace glnav
{
    template<typename T, typename Q>
    class astar
    {
    public:
        astar(const maze<T, Q> &seed)
            : __maze(seed),
            __solved(false)
        {
            this->reset();
        }

        virtual ~astar()
        { }

        void iterate()
        {
            if(!this->__maze.is_valid()) throw std::runtime_error("Invalid maze");

            if(this->__open_set.size() == 0) {
                this->__solved = true;
                return;
            }

            // Get the next point 
            point<T> current = this->__open_set.front();
            double current_cost = this->__fscore.cost(current);
            assert(current_cost < std::numeric_limits<double>::infinity());
            for(size_t i = 1; i < this->__open_set.size(); i++)
            {
                const double test_cost = this->__fscore.cost(this->__open_set.at(i));
                if(test_cost < current_cost) {
                    current = this->__open_set.at(i);
                    current_cost = test_cost;
                }
            }
            this->__open_set.remove(current);

            // Check for completion
            if(current == this->__maze.finish()) {
                this->__solved = true;
                return;
            }

            const neighborhood<T, Q> neighbors = this->__maze.neighbors(current);
            assert(neighbors.size() > 0);
            for(size_t i = 0; i < neighbors.size(); i++)
            {
                const Q tentative = this->__gscore.cost(current) + neighbors.at(i).cost;
                const point<T> neighbor = neighbors.at(i).location();
                if(tentative < this->__gscore.cost(neighbor))
                {
                    this->__gscore.set(neighbor, tentative);
                    this->__fscore.set(neighbor, tentative + this->__heuristic(neighbor));
                    if(!this->__open_set.contains(neighbor)) this->__open_set.push_back(neighbor);
                }
            }
        }

        bool is_solved() const { return this->__solved; }

        route<T, Q> build_route() const
        {
            route<T, Q> result;
            if(!this->is_solved()) return result;

            point<T> current = this->__maze.finish();

            while(current != this->__maze.start())
            {
                const neighborhood<T, Q> neighbors = this->__maze.neighbors(current);
                assert(neighbors.size() > 0);
                point<T> next = neighbors.at(0).location();
                double cost = this->__gscore.cost(next);
                for(size_t i = 1; i < neighbors.size(); i++)
                {
                    if(this->__gscore.cost(neighbors.at(i).location()) < cost) {
                        next = neighbors.at(i).location();
                        cost = this->__gscore.cost(next);
                    }
                }

                // Check for loop
                for(size_t i = 0; i < result.size(); i++)
                {
                    if(result[i].target == next) throw std::runtime_error("Loop detected");
                }

                // Set the speed
                const Q delta_cost = this->__gscore.cost(current) - this->__gscore.cost(next);
                const path<T> next_path = path<T>(current, next);
                const Q distance = glnav::length<T, Q>(next_path);

                // Set the new waypoint
                result.push_front(heading<T, Q>(current, distance / delta_cost));
                
                // Move the points
                current = next;
            }

            return result;
        }

        void reset()
        {
            if(!this->__maze.is_valid()) throw std::runtime_error("Invalid maze");
            this->__gscore = cost_map<T, Q>(this->__maze);
            this->__gscore.set(this->__maze.start(), 0.0);
            this->__fscore = cost_map<T, Q>(this->__maze);
            this->__fscore.set(this->__maze.start(), this->__heuristic(this->__maze.start()));
            this->__open_set.clear();
            this->__open_set.push_back(this->__maze.start());
            this->__solved = false;
        }
    private:
        Q __heuristic(const point<T> &point)
        {
            return magnitude<T, Q>(point - this->__maze.finish());
        }

        const maze<T, Q> &__maze;
        cost_map<T, Q> __gscore;
        cost_map<T, Q> __fscore;
        point_group<T> __open_set;
        bool __solved;
    };
}

#endif