#ifndef GLNAV_DIJKSTRA_H
#define GLNAV_DIJKSTRA_H

#include "glnav_network.h"
#include "glnav_cost_map.h"
#include <stdexcept>

namespace glnav
{
    template<typename T>
    class dijkstra
    {
    public:
        dijkstra(network<T> &net, const point<T> &from, const point<T> &to)
            : __net(net),
            __from(from),
            __to(to),
            __solved(false)
        {
            this->reset();
        }

        virtual ~dijkstra()
        { }

        size_t iterate()
        {
            // Check for version update
            if(this->__net.version() != this->__version) this->reset();

            size_t num_changes = this->__map.iterate();
            if(num_changes == 0) this->__solved = true;
            return num_changes;
        }

        bool is_solved() const { return this->__solved; }

        point_group<T> route() const
        {
            point_group<T> result;
            if(!this->__solved) return result;
            assert(this->__net.contains(this->__from));
            assert(this->__net.contains(this->__to));
            if(this->__map.cost(this->__from) == std::numeric_limits<double>::infinity()) return result;
            result.push_back(this->__from);
            while(result.back() != this->__to)
            {
                const neighborhood<T, double> neighbors = this->__net.neighbors(result.back());
                assert(neighbors.size() > 0);
                point<T> next = neighbors.at(0);
                double cost = this->__map.cost(next);
                for(size_t i = 1; i < neighbors.size(); i++)
                {
                    if(this->__map.cost(neighbors.at(i)) < cost) {
                        next = neighbors.at(i);
                        cost = this->__map.cost(next);
                    }
                }
                if(result.contains(next)) throw std::runtime_error("Loop detected");
                result.push_back(next);
                assert(result.size() < this->__net.size());
            }
            return result;
        }

        void reset()
        {
            if(!this->__net.contains(this->__from)) throw std::runtime_error("Network does not contain origin");
            if(!this->__net.contains(this->__to)) throw std::runtime_error("Network does not contain destination");
            this->__map = cost_map<T>(this->__net);
            this->__map.set(this->__to, 0.0);
            this->__solved = false;
            this->__version = this->__net.version();
        }
    private:
        network<T> &__net;
        cost_map<T> __map;
        point<T> __from;
        point<T> __to;
        bool __solved;
        version_t __version;
    };
}

#endif