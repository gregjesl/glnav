#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include "glnav_version_control.h"
#include "glnav_neighbor.h"
#include <assert.h>
#include <map>
#include <inttypes.h>

namespace glnav
{

    template<typename T>
    class neighbor_map : private std::map<const point<T> * const, double>
    {
    public:
        neighbor_map(const point<T> &origin)
            : std::map<const point<T> * const, double>(),
            __origin(origin)
        { }

        void update(const point<T> * const neighbor, const double cost)
        {
            typename std::map<const point<T> * const, double>::iterator it = this->find(neighbor);
            if(it == this->end())
            {
                this->insert(
                    std::pair<const point<T> * const, double>(neighbor, cost)
                );
            }
            else
            {
                it->second = cost;
            }
        }

        const point<T> & origin() const { return this->__origin; }

        using std::map<const point<T> * const, double>::erase;

        double cost(const point<T> * const to) const
        {
            typename std::map<const point<T> * const, double>::const_iterator it = this->find(to);
            return it != this->end() ? it->second : std::numeric_limits<double>::infinity();
        }

        neighborhood<T, double> neighbors() const
        {
            neighborhood<T, double> result(this->__origin);
            typename std::map<const point<T> * const, double>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                result.push_back(
                    neighbor<T, double>(*it->first, it->second)
                    );
            }
            return result;
        }

    private:
        point<T> __origin;
    };

    #define network_base_t point_map<T, neighbor_map<T> >
    typedef uint64_t version_t;

    template<typename T>
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
                    std::pair<point<T>, neighbor_map<T> >(it->first, neighbor_map<T>(it->first))
                );
            }

            // Copy the paths
            for(it = other.begin(); it != other.end(); ++it)
            {
                neighborhood<T, double> neighbors = it->second.neighbors();
                for(size_t i = 0; i < neighbors.size(); i++)
                {
                    typename network_base_t::iterator internal = this->find(neighbors.at(i));
                    assert(internal != this->end());
                    internal->second.update(&internal->first, neighbors.at(i).cost);
                }
            }

            // Iterate the version
            this->update_version();

            // Test
            assert(this->size() == other.size());

            return *this;
        }
        
        void add(const path<T> &seed, const double cost)
        {
            this->__add(seed.start, seed.end, cost);
            this->__add(seed.end, seed.start, cost);
            this->update_version();
        }

        void remove(const point<T> &node)
        {
            typename network_base_t::iterator it = this->find(node);
            if(it == this->end()) return;
            const point<T> * end_ptr = &it->first;
            neighborhood<T, double> neighbors = it->second.neighbors();
            for(size_t i = 0; i < neighbors.size(); i++)
            {
                typename network_base_t::iterator it = this->find(neighbors.at(i));
                assert(it != this->end());
                it->second.erase(end_ptr);
            }
            this->erase(node);
            this->update_version();
        }

        using network_base_t::size;
        using network_base_t::empty;
        using network_base_t::contains;
        using network_base_t::overlap;

        double cost(const point<T> &from, const point<T> &to) const
        {
            typename network_base_t::const_iterator start_it = this->find(from);
            if(start_it == this->end()) throw unknown_point_exception<T>(from);

            typename network_base_t::const_iterator end_it = this->find(to);
            if(end_it == this->end()) throw unknown_point_exception<T>(to);
            const point<T> * end_ptr = &end_it->first;

            return start_it->second.cost(end_ptr);
        }

        neighborhood<T, double> neighbors(const point<T> &node) const
        {
            neighborhood<T, double> result(node);
            typename network_base_t::const_iterator start_it = this->find(node);
            if(start_it == this->end()) throw unknown_point_exception<T>(node);
            return start_it->second.neighbors();
        }

        std::map<point<T>, double> node_map(const double seed) const
        {
            std::map<point<T> , double> result;
            typename network_base_t::const_iterator it;
            for(it = this->begin(); it != this->end(); it++)
            {
                result.insert(
                    std::pair<point<T>, double>(it->first, seed)
                );
            }
            return result;
        }

        point_group<T> overlap(const network<T> &other) const
        {
            return network_base_t::overlap(other);
        }

    private:
        void __add(const point<T> &start, const point<T> &end, const double cost)
        {
            typename network_base_t::iterator start_it = this->find(start);
            while(start_it == this->end())
            {
                this->insert(
                        std::pair<
                            point<T>,
                            neighbor_map<T>
                        >
                        (
                            start,
                            neighbor_map<T>(start)
                        )
                    );
                start_it = this->find(start);
            }

            typename network_base_t::iterator end_it = this->find(end);
            while(end_it == this->end())
            {
                this->insert(
                        std::pair<
                            point<T>,
                            neighbor_map<T>
                        >
                        (
                            end,
                            neighbor_map<T>(end)
                        )
                    );
                end_it = this->find(end);
            }
            const point<T> *end_ptr = &end_it->first;

            assert(start_it != this->end());
            assert(end_ptr != nullptr);
            start_it->second.update(end_ptr, cost);
        }
    };

    #undef network_base_t
}

#endif