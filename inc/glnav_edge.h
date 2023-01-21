#ifndef GLNAV_EDGE_H
#define GLNAV_EDGE_H

#include "glnav_point.h"
#include <utility>
#include <assert.h>

namespace glnav
{
    template<typename T>
    class edge : public std::pair<const point<T> *, const point<T> *>
    {
    public:
        edge(const point<T> * first, const point<T> * second)
            : std::pair<const point<T> *, const point<T> *>(first < second ? first : second, first > second ? first : second)
        { 
            assert(first != nullptr);
            assert(second != nullptr);
        }

        edge(const edge &other)
            : std::pair<const point<T> *, const point<T> *>(other)
        { 
            assert(this->first != nullptr);
            assert(this->second != nullptr);
        }

        edge& operator=(const edge &other)
        {
            this->first = other.first;
            this->second = other.second;
            
            assert(this->first != nullptr);
            assert(this->second != nullptr);
        }

        virtual ~edge() { }

        bool operator==(const edge &other)
        {
            assert(this->first != nullptr);
            assert(this->second != nullptr);
            assert(this->first <= this->second);
            assert(other.first <= other.second);

            return this->first == other.first && this->second == other.second;
        }

        bool operator!=(const edge &other)
        {
            assert(this->first != nullptr);
            assert(this->second != nullptr);
            assert(this->first <= this->second);
            assert(other.first <= other.second);

            return this->first != other.first || this->second != other.second;
        }

        bool operator<(const edge &other)
        {
            assert(this->first != nullptr);
            assert(this->second != nullptr);
            assert(this->first <= this->second);
            assert(other.first <= other.second);

            if(this->first < other.first) return true;
            if(this->first > other.first) return false;
            assert(this->first == other.first);
            return this->second < other.second;
        }

        bool operator>(const edge &other)
        {
            assert(this->first != nullptr);
            assert(this->second != nullptr);
            assert(this->first <= this->second);
            assert(other.first <= other.second);

            if(this->first > other.first) return true;
            if(this->first < other.first) return false;
            assert(this->first == other.first);
            return this->second > other.second;
        }

        path<T> as_path() const
        {
            return path<T>(*this->first, *this->second);
        }
    };
}

#endif