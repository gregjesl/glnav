#ifndef GLNAV_LEVEL_H
#define GLNAV_LEVEL_H

#include "glnav_traveler.h"
#include "glnav_obstacle.h"
#include "glnav_network.h"
#include <set>
#include <map>

namespace glnav
{
    template<typename T, typename Q>
    class level
    {
    public:
        level()
        { }

        void attach(travler<T, Q> &tvlr)
        {
            this->__travelers.add(tvlr);
        }

        void detatch(travler<T, Q> &tvlr)
        {
            this->__travelers.remove(tvlr);
        }

        void run(const Q duration)
        {
            // Purge travelers
            this->__travelers.purge();

            if(this->__obstacles.synchronize())
            {
                // The network has changed
                this->__rebuild_level();
            }

            // Merge in travelers
            if(this->__travelers.merge())
            {

            }

            // Move travelers
            this->__propogate_travelers(duration);
        }

    private:
        dynamic_set<obstacle<T, Q> *> __obstacles;
        dynamic_set<traveler<T, Q> *> __travelers;
        network<T, Q> __net;

        void __rebuild_level()
        {
            // Rebuild the network

            // Update all the travelers
            typename std::set<traveler<T, Q> *>::iterator it;
            for(it = this->__travelers.values.begin(); it != this->__travelers.values.end(); ++it)
                this->__remap_traveler(*it);
        }

        void __remap_traveler(traveler<T, Q> * tvlr)
        {

        }

        void __propogate_travelers(const Q duration)
        {
            if(this->__travelers.values.empty()) return;

            typename std::set<traveler<T, Q> *>::iterator it;
            for(it = this->__travelers.values.begin(); it != this->__travelers.values.end(); ++it)
            {
                it->travel(duration);
            }
        }
    };
}

#endif