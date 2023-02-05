#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include "glnav_version_control.h"
#include "glnav_neighbor.h"
#include "glnav_map.h"
#include <assert.h>
#include <map>
#include <inttypes.h>

namespace glnav
{
    #define network_base_t point_map<T, neighborhood<T, Q> >
    typedef uint64_t version_t;

    template<typename T, typename Q>
    class network : private network_base_t,
        public version_controlled
    {
    public:
        network()
            : network_base_t(),
            version_controlled()
        { }

        network(const network &other)
            : network_base_t(),
            version_controlled()
        {
            this->operator=(other);
        }

        network& operator=(const network &other)
        {
            this->clear();

            // Populate all the base points
            typename network_base_t::const_iterator it;
            for(it = other.begin(); it != other.end(); ++it)
            {
                this->insert(
                    std::pair<point<T>, neighborhood<T, Q> >(it->first, neighborhood<T, Q>())
                );
            }

            // Copy the paths
            for(it = other.begin(); it != other.end(); ++it)
            {
                const neighborhood<T, Q> &neighbors = it->second;
                for(size_t i = 0; i < neighbors.size(); i++)
                {
                    typename network_base_t::iterator internal = this->find(neighbors.at(i).location());
                    assert(internal != this->end());
                    internal->second.push_back(neighbor<T, Q>(internal->first, neighbors.at(i).cost));
                }
            }

            // Iterate the version
            this->update_version();

            // Test
            assert(this->size() == other.size());

            return *this;
        }
        
        void add(const path<T> &seed, const Q cost)
        {
            this->__add(seed.start, seed.end, cost);
            this->__add(seed.end, seed.start, cost);
            this->update_version();
        }

        void remove(const point<T> &node)
        {
            typename network_base_t::iterator it = this->find(node);
            if(it == this->end()) return;

            // Purge the node from each of the neighbors
            {
                const neighborhood<T, Q> &neighbors = it->second;
                for(size_t i = 0; i < neighbors.size(); i++)
                {
                    typename network_base_t::iterator it = this->find(neighbors.at(i).location());
                    assert(it != this->end());
                    neighborhood<T, Q> update;
                    for(size_t i = 0; i < it->second.size(); i++)
                    {
                        if(it->second.at(i).is_located_at(node))
                            continue;
                        
                        update.push_back(it->second.at(i));
                    }
                    it->second = update;
                }
            }

            // Purge the node itself
            this->erase(node);

            this->update_version();
        }

        using network_base_t::size;
        using network_base_t::empty;
        using network_base_t::contains;

        network<T, Q> & from_map(map<T> &seed, const point<T> &start, const point<T> &finish)
        {
            if(start == finish) throw std::invalid_argument("Starting point is the same as the finishing point");
            std::set<point<T> > nodes;
            std::set<obstacle<T> *> obstacles;
            nodes.insert(start);
            nodes.insert(finish);
            size_t num_nodes = 0;
            while(num_nodes != nodes.size())
            {
                num_nodes = nodes.size();
                obstacles = seed.obstacles(nodes);
                nodes = corners(obstacles);
                nodes.insert(start);
                nodes.insert(finish);
            }

            std::vector<path<T> > paths = seed.paths(nodes);
            this->clear();
            for(size_t i = 0; i < paths.size(); i++)
            {
                this->add(paths.at(i), paths.at(i).cost());
            }
            this->update_version();
        }

        /*
        double cost(const point<T> &from, const point<T> &to) const
        {
            typename network_base_t::const_iterator start_it = this->find(from);
            if(start_it == this->end()) throw unknown_point_exception<T>(from);

            typename network_base_t::const_iterator end_it = this->find(to);
            if(end_it == this->end()) throw unknown_point_exception<T>(to);
            const point<T> * end_ptr = &end_it->first;

            return start_it->second.cost(end_ptr);
        }
        */

        const neighborhood<T, Q>& neighbors(const point<T> &node) const
        {
            typename network_base_t::const_iterator it = this->find(node);
            if(it == this->end()) throw unknown_point_exception<T>(node);
            return it->second;
        }

        bool contains(const path<T> &input) const
        {
            if(!this->contains(input.start) || !this->contains(input.end)) return false;
            return this->neighbors(input.start).contains(input.end);
        }

        std::map<point<T>, Q> node_map(const Q seed) const
        {
            std::map<point<T> , Q> result;
            typename network_base_t::const_iterator it;
            for(it = this->begin(); it != this->end(); it++)
            {
                result.insert(
                    std::pair<point<T>, Q>(it->first, seed)
                );
            }
            return result;
        }

        point_group<T> overlap(const network<T, Q> &other) const
        {
            return network_base_t::overlap(other);
        }

    private:
        void __add(const point<T> &start, const point<T> &end, const Q cost)
        {
            typename network_base_t::iterator start_it = this->find(start);
            while(start_it == this->end())
            {
                this->insert(
                        std::pair<
                            point<T>,
                            neighborhood<T, Q>
                        >
                        (
                            start,
                            neighborhood<T, Q>()
                        )
                    );
                start_it = this->find(start);
            }
            assert(start_it != this->end());

            typename network_base_t::iterator end_it = this->find(end);
            while(end_it == this->end())
            {
                this->insert(
                        std::pair<
                            point<T>,
                            neighborhood<T, Q>
                        >
                        (
                            end,
                            neighborhood<T, Q>()
                        )
                    );
                end_it = this->find(end);
            }
            assert(end_it != this->end());

            if(start_it->second.contains(end_it->first)) throw std::runtime_error("Attempted to add an existing path");
            const neighbor<T, Q> next(end_it->first, cost);
            start_it->second.push_back(next);
        }
    };

    #undef network_base_t
}

#endif