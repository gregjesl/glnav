#ifndef GLNAV_NODE_H
#define GLNAV_NODE_H

#include "glnav_point.h"

namespace glnav
{
    template<typename T>
    class node
    {
    public:
        node(const point<T> &location)
            : __location(&location)
        { }

        node(const node &other)
            : __location(other.__location)
        { }

        node& operator=(const node &other)
        {
            this->__location = other.__location;
            return *this;
        }

        bool is_located_at(const point<T> &location) const
        {
            return this->__location->operator==(location);
        }

        const point<T> & location() const { return *this->__location; }

    private:
        const point<T> * __location;
    };
}

#endif