#ifndef GLNAV_LEVEL_H
#define GLNAV_LEVEL_H

#include "glnav_traveler.h"
#include "glnav_obstacle.h"
#include "glnav_network.h"
#include <set>
#include <map>

namespace glnav
{
    template<typename T>
    class dynamic_set
    {
    public:
        typedef enum action_enum
        {
            ADD,
            REMOVE
        } action_t;

        void add(T value)
        {
            typename std::map<T, action_t>::iterator it = this->__changes.find(value);
            if(it != this->__changes.end())
            {
                it->second = ADD;
                return;
            }
            this->__changes.insert(
                std::pair<T, action_t>(value, ADD)
            );
        }

        void remove(T value)
        {
            typename std::map<T, action_t>::iterator it = this->__changes.find(value);
            if(it != this->__changes.end())
            {
                it->second = REMOVE;
                return;
            }
            this->__changes.insert(
                std::pair<T, action_t>(value, REMOVE)
            );
        }

        bool synchronize()
        {
            if(this->__changes.empty()) return false;

            while(!this->__changes.empty())
            {
                typename std::map<T, action_t>::iterator it = this->__changes.begin();
                switch (it->second)
                {
                case ADD:
                    this->values.insert(it->first);
                    break;
                case REMOVE:
                    this->values.erase(it->first);
                }
                this->__changes.erase(it);
            }
            return true;
        }

        size_t pending_changes() const { return this->__changes.size(); }

        std::set<T> values;
    private:
        std::map<T, action_t> __changes;
    };

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
            this->__travelers.synchronize();
            assert(this->__travelers.pending_changes() == 0);

            if(this->__obstacles.synchronize())
            {
                // The network has changed
                this->__rebuild_level();
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