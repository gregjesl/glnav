#ifndef GLNAV_NETWORK_H
#define GLNAV_NETWORK_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include <assert.h>
#include <map>

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

    #define network_value_t std::pair<neighbor_map<T>, Q>
    #define network_base_t std::map<const point<T>,  network_value_t>

    template<typename T, typename Q>
    class network : private network_base_t
    {
    public:
        network()
            : network_base_t()
        { }
        
        void add(const path<T> &seed, const double cost, Q metadata)
        {
            this->__add(seed.start, seed.end, cost, metadata);
            this->__add(seed.end, seed.start, cost, metadata);
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

            return start_it->second.first.cost(end_ptr);
        }

        std::vector<std::pair<point<T>, double> > neighbors(const point<T> &node) const
        {
            std::vector<std::pair<point<T>, double> > result;
            typename network_base_t::const_iterator start_it = this->find(node);
            if(start_it == this->end()) return result;
            return start_it->second.first.neighbors();
        }

        Q* metadata(const point<T> &node)
        {
            typename network_base_t::iterator start_it = this->find(node);
            if(start_it == this->end()) return nullptr;
            return &start_it->second.second;
        }

    private:
        void __add(const point<T> &start, const point<T> &end, const double cost, Q metadata)
        {
            typename network_base_t::iterator start_it = this->find(start);
            while(start_it == this->end())
            {
                this->insert(
                        std::pair<
                            point<T>,
                            network_value_t
                        >
                        (
                            start,
                            network_value_t()
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
                            network_value_t
                        >
                        (
                            end,
                            network_value_t()
                        )
                    );
                end_it = this->find(end);
            }
            const point<T> *end_ptr = &end_it->first;

            assert(start_it != this->end());
            assert(end_ptr != nullptr);
            start_it->second.first.update(end_ptr, cost);
            start_it->second.second = metadata;
        }
    };

    #undef network_base_t
    #undef network_value_t
}

#endif