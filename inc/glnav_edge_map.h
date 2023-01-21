#ifndef GLNAV_EDGE_MAP_H
#define GLNAV_EDGE_MAP_H

#include "glnav_point.h"
#include "glnav_edge.h"
#include <map>
#include <vector>

#define edge_map_base_t std::map<const point<T> *, std::vector<const point<T> * > >

namespace glnav
{
    template<typename T>
    class edge_map : private edge_map_base_t
    {
    public:
        edge_map()
            : edge_map_base_t()
        { }

        void add(const point<T> *start, const point<T> *end)
        {
            this->__add(start, end);
            this->__add(end, start);
        }

        void add(const edge<T> &input)
        {
            this->__add(input.first, input.second);
            this->__add(input.second, input.first);
        }

        using edge_map_base_t::size;

        std::vector<const point<T> * > neighbors(const point<T> *start) const
        {
            typename edge_map_base_t::const_iterator it = this->find(start);
            if(it == this->end())
                return std::vector<const point<T> * >();
            return it->second;
        }

        std::vector<edge<T> > edges(const point<T> *start) const
        {
            typename edge_map_base_t::const_iterator it = this->find(start);
            if(it == this->end())
                return std::vector<edge<T> >();
            std::vector<edge<T> > result;
            for(size_t i = 0; i < it->second.size(); i++)
            {
                result.push_back(edge<T>(start, it->second.at(i)));
            }
            return result;
        }

    private:
        void __add(const point<T> *start, const point<T> *end)
        {
            typename edge_map_base_t::iterator it = this->find(start);
            if(it == this->end())
            {
                this->insert(
                    std::pair<
                        const point<T> *,
                        std::vector<const point<T> *>
                    >(
                        start,
                        std::vector<const point<T> *>(1, end)
                    )
                );
            }
            else
            {
                for(size_t i = 0; i < it->second.size(); i++)
                {
                    if(it->second.at(i) == end) {
                        // Already contained
                        return;
                    }
                }
                it->second.push_back(end);
            }
        }
    };
}

#endif