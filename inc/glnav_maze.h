#ifndef GLNAV_MAZE_H
#define GLNAV_MAZE_H

#include "glnav_network.h"
#include "glnav_cost_map.h"
#include "glnav_version_control.h"

namespace glnav
{
    /*! \brief Network with a defined start and end
     *
     * The maze class is designed such multiple mazes can share one common network
     */
    template<typename T, typename Q>
    class maze : public network<T, Q>
    {
    public:
        maze(const point<T> &start, 
            const point<T> &finish)
            : network<T, Q>(),
            __start(start),
            __finish(finish)
        { }

        virtual ~maze()
        { }

        const point<T> & start() const { return this->__start; }
        const point<T> & finish() const { return this->__finish; }
        bool is_valid() const { return this->contains(this->__start) && this->contains(this->__finish); }

    private:
        point<T> __start;
        point<T> __finish;
    };
}

#endif