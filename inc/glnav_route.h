#ifndef GLNAV_ROUTE_H
#define GLNAV_ROUTE_H

#include "glnav_neighbor.h"
#include "glnav_network.h"
#include "glnav_heading.h"
#include "glnav_version_control.h"
#include <deque>

namespace glnav
{
    template<typename T, typename Q>
    class route : private std::deque<heading<T, Q> >, public version_dependent
    {
    public:
        route(const network<T, Q> &seed)
            : std::deque<heading<T, Q> >(),
            version_dependent(seed),
            __seed(seed)
        { }

        using std::deque<heading<T, Q> >::empty;
        using std::deque<heading<T, Q> >::size;
        using std::deque<heading<T, Q> >::clear;
        using std::deque<heading<T, Q> >::operator[];

        void push_front(const point<T> & waypoint, const Q speed)
        {
            assert_version(*this, this->__seed);

            std::deque<heading<T, Q> >::push_front(
                heading<T, Q>(waypoint, speed)
            );
        }

        void push_back(const point<T> & waypoint, const Q speed)
        {
            assert_version(*this, this->__seed);

            std::deque<heading<T, Q> >::push_back(
                heading<T, Q>(waypoint, speed)
            );
        }

        heading<T, Q> pop_front()
        {
            assert_version(*this, this->__seed);

            return std::deque<heading<T, Q> >::pop_front();
        }

        heading<T, Q> pop_back()
        {
            assert_version(*this, this->__seed);

            return std::deque<heading<T, Q> >::pop_back();
        }

        heading<T, Q> & front()
        {
            assert_version(*this, this->__seed);

            return std::deque<heading<T, Q> >::front();
        }

        heading<T, Q> & back()
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::back();
        }

        const heading<T, Q> & front() const
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::front();
        }

        const heading<T, Q> & back() const
        {
            assert_version(*this, this->__seed);

            return std::deque<std::pair<point<T>, Q> >::back();
        }
    
    private:
        const network<T, Q> & __seed;
    };
}

#endif