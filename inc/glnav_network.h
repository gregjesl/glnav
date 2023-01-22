#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include "glnav_cost_map.h"
#include <assert.h>
#include <map>
#include <inttypes.h>

namespace glnav
{
    template<typename T>
    class neighbor_map : private std::map<const point<T> * const, double>
    {
    public:
        neighbor_map()
            : std::map<const point<T> * const, double>()
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

        using std::map<const point<T> * const, double>::erase;

        double cost(const point<T> * const to) const
        {
            typename std::map<const point<T> * const, double>::const_iterator it = this->find(to);
            return it != this->end() ? it->second : std::numeric_limits<double>::infinity();
        }

        std::vector<std::pair<point<T>, double> > neighbors() const
        {
            std::vector<std::pair<point<T>, double> > result;
            typename std::map<const point<T> * const, double>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                result.push_back(
                    std::pair<point<T>, double>(*it->first, it->second)
                    );
            }
            return result;
        }
    };

    #define network_base_t std::map<const point<T>, neighbor_map<T> >

    template<typename T>
    class network : private network_base_t
    {
    public:
        network()
            : network_base_t(),
            __version(0)
        { }
        
        void add(const path<T> &seed, const double cost)
        {
            if(this->__version == UINT64_MAX) throw std::overflow_error("Version overflow");
            this->__add(seed.start, seed.end, cost);
            this->__add(seed.end, seed.start, cost);
            this->__version++;
        }

        void remove(const point<T> &node)
        {
            if(this->__version == UINT64_MAX) throw std::overflow_error("Version overflow");
            typename network_base_t::iterator it = this->find(node);
            if(it == this->end()) return;
            const point<T> * end_ptr = &it->first;
            std::vector<std::pair<point<T>, double> > neighbors = it->second.neighbors();
            for(size_t i = 0; i < neighbors.size(); i++)
            {
                typename network_base_t::iterator it = this->find(neighbors.at(i).first);
                assert(it != this->end());
                it->second.erase(end_ptr);
            }
            this->erase(node);
            this->__version++;
        }

        bool contains(const point<T> &node) const
        {
            return this->find(node) != this->end();
        }

        using network_base_t::size;

        double cost(const point<T> &from, const point<T> &to) const
        {
            typename network_base_t::const_iterator start_it = this->find(from);
            if(start_it == this->end()) return std::numeric_limits<double>::infinity();

            typename network_base_t::const_iterator end_it = this->find(to);
            if(end_it == this->end()) return std::numeric_limits<double>::infinity();
            const point<T> * end_ptr = &end_it->first;

            return start_it->second.cost(end_ptr);
        }

        std::vector<std::pair<point<T>, double> > neighbors(const point<T> &node) const
        {
            std::vector<std::pair<point<T>, double> > result;
            typename network_base_t::const_iterator start_it = this->find(node);
            if(start_it == this->end()) return result;
            return start_it->second.neighbors();
        }

        template<typename W>
        std::map<point<T> , W> node_map(const W seed) const
        {
            std::map<point<T> , W> result;
            typename network_base_t::const_iterator it;
            for(it = this->begin(); it != this->end(); it++)
            {
                result.insert(
                    std::pair<point<T>, W>(it->first, seed)
                );
            }
            return result;
        }

        cost_map<T> seed_cost_map() const
        {
            cost_map<T> result;
            typename network_base_t::const_iterator it;
            for(it = this->begin(); it != this->end(); it++)
            {
                result.seed(it->first);
            }
            return result;
        }

        point_group<T> overlap(const network<T> &other) const
        {
            point_group<T> result;
            typename network_base_t::const_iterator core;
            for(core = this->begin(); core != this->end(); ++core)
            {
                if(other.contains(core->first)) {
                    result.push_back(core->first);
                }
            }
            return result;
        }

        uint64_t version() const { return this->__version; }

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
                            neighbor_map<T>()
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
                            neighbor_map<T>()
                        )
                    );
                end_it = this->find(end);
            }
            const point<T> *end_ptr = &end_it->first;

            assert(start_it != this->end());
            assert(end_ptr != nullptr);
            start_it->second.update(end_ptr, cost);
        }

        uint64_t __version;
    };

    #undef network_base_t
}

#endif