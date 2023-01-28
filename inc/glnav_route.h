#ifndef GLNAV_ROUTE_H
#define GLNAV_ROUTE_H

#include "glnav_neighbor.h"
#include "glnav_network.h"
#include "glnav_version_control.h"
#include <deque>

namespace glnav
{
    template<typename T, typename Q>
    class route : private std::deque<std::pair<point<T>, Q> >, public version_dependent
    {
    public:
        route(const network<T, Q> &seed)
            : std::deque<std::pair<point<T>, Q> >(),
            version_dependent(seed),
            __seed(seed)
        { }

        using std::deque<std::pair<point<T>, Q> >::empty;
        using std::deque<std::pair<point<T>, Q> >::size;
        using std::deque<std::pair<point<T>, Q> >::clear;
        using std::deque<std::pair<point<T>, Q> >::operator[];

        void push_front(const point<T> & waypoint, const Q speed)
        {
            assert_version(*this, this->__seed);

            std::deque<std::pair<point<T>, Q> >::push_front(
                std::pair<point<T> , Q>(waypoint, speed)
            );
        }

        void push_back(const point<T> & waypoint, const Q speed)
        {
            assert_version(*this, this->__seed);

            std::deque<std::pair<point<T>, Q> >::push_back(
                std::pair<point<T> , Q>(waypoint, speed)
            );
        }

        std::pair<point<T>, Q> pop_front()
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::pop_front();
        }

        std::pair<point<T>, Q> pop_back()
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::pop_back();
        }

        std::pair<point<T>, Q> & front()
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::front();
        }

        std::pair<point<T>, Q> & back()
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::back();
        }

        const std::pair<point<T>, Q> & front() const
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::front();
        }

        const std::pair<point<T>, Q> & back() const
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::back();
        }
    
    private:
        const network<T, Q> & __seed;
    };
}

#endif