#ifndef GLNAV_ASTAR_H
#define GLNAV_ASTAR_H

#include "glnav_network.h"
#include <algorithm>

namespace glnav
{
    template<typename T>
    class astar
    {
    public:
        astar(network<T> &net, const point<T> &from, const point<T> &to)
            : __net(net),
            __path(),
            __from(from),
            __to(to),
            __solved(false)
        {
            this->reset();
        }

        virtual ~astar()
        { }

        void iterate()
        {
            // Check for version update
            if(this->__net.version() != this->__version) this->reset();

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
            if(current == this->__to) {
                this->__solved = true;
                return;
            }

            const std::vector<std::pair<point<T>, double> > neighbors = this->__net.neighbors(current);
            assert(neighbors.size() > 0);
            for(size_t i = 0; i < neighbors.size(); i++)
            {
                const double tentative = this->__gscore.cost(current) + neighbors.at(i).second;
                const point<T> &neighbor = neighbors.at(i).first;
                if(tentative < this->__gscore.cost(neighbor))
                {
                    this->__path.set(neighbor, current);
                    this->__gscore.set(neighbor, tentative);
                    this->__fscore.set(neighbor, tentative + this->__heuristic(neighbor));
                    if(!this->__open_set.contains(neighbor)) this->__open_set.push_back(neighbor);
                }
            }
        }

        bool is_solved() const { return this->__solved; }

        point_group<T> route() const
        {
            point_group<T> result;
            if(!this->__solved) return result;
            assert(this->__net.contains(this->__from));
            assert(this->__net.contains(this->__to));
            if(!this->__path.contains(this->__to)) return result;
            point<T> current = this->__to;
            while(current != this->__from)
            {
                result.push_back(current);
                const point<T> * next = this->__path.get(current);
                assert(next != nullptr);
                current = *next; 
            }
            result.push_back(this->__from);
            std::reverse(result.begin(), result.end());
            return result;
        }

        void reset()
        {
            if(!this->__net.contains(this->__from)) throw std::runtime_error("Network does not contain origin");
            if(!this->__net.contains(this->__to)) throw std::runtime_error("Network does not contain destination");
            this->__gscore = cost_map<T>(this->__net);
            this->__gscore.set(this->__from, 0.0);
            this->__fscore = cost_map<T>(this->__net);
            this->__fscore.set(this->__from, this->__heuristic(this->__from));
            this->__open_set.clear();
            this->__open_set.push_back(this->__from);
            this->__solved = false;
            this->__version = this->__net.version();
        }
    private:
        double __heuristic(const point<T> &point)
        {
            return (point - this->__to).magnitude();
        }

        network<T> &__net;
        cost_map<T> __gscore;
        cost_map<T> __fscore;
        point_group<T> __open_set;
        point_map<T, point<T> > __path;
        point<T> __from;
        point<T> __to;
        bool __solved;
        version_t __version;
    };
}

#endif