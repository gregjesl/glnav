#ifndef GLNAV_COST_MAP_H
#define GLNAV_COST_MAP_H

#include "glnav_point.h"
#include "glnav_network.h"
#include "glnav_version_control.h"
#include "glnav_neighbor.h"
#include <map>

namespace glnav
{
    template<typename T, typename Q>
    class cost_map : private point_map<T, Q>,
        public version_dependent
    {
    public:
        cost_map() 
            : point_map<T, Q>(),
            version_dependent(),
            __net(nullptr)
        { }

        cost_map(const network<T, Q> &net)
            : point_map<T, Q>(net.node_map(std::numeric_limits<Q>::infinity())),
            version_dependent(net),
            __net(&net)
        { }

        cost_map& operator=(const cost_map<T, Q> &other)
        {
            point_map<T, Q>::operator=(other);
            if(other.__net != nullptr)
                this->attach(*other.__net);
            return *this;
        }

        using point_map<T, Q>::size;
        using point_map<T, Q>::empty;
        using point_map<T, Q>::contains;
        
        Q cost(const point<T> &input) const
        {
            // Perform version check
            if(this->__net != nullptr && !this->versions_synchronized(*this->__net))
                throw version_mismatch(*this, *this->__net);
            assert(this->contains(input));
            return this->force_get(input);
        }

        bool set(const point<T> &input, const Q value)
        {
            // Perform version check
            if(this->__net != nullptr)
            {
                if(this->__net != nullptr && !this->versions_synchronized(*this->__net))
                    throw version_mismatch(*this, *this->__net);
                
                if(!this->__net->contains(input))
                    throw unknown_point_exception<T>(input);
            }

            return point_map<T, Q>::set(input, value); // Returns true if new
        }

        void attach(const glnav::network<T, Q> &net)
        {
            if(this->__net != nullptr)
                throw std::runtime_error("Already attached to a network");
            
            if(!this->empty())
            {
                typename std::map<point<T> , Q>::const_iterator it;
                for(it = this->begin(); it != this->end(); ++it)
                {
                    if(!net.contains(it->first))
                        throw std::runtime_error("Orphaned point");
                }
            }

            this->__net = &net;
            this->set_version(this->__net->version());
        }

        void detatch()
        {
            this->__net = nullptr;
            this->set_version(0);
        }

        const network<T, Q>& network() const { return this->__net; }
        bool is_attached() const { return this->__net != nullptr; }
        bool is_syncronized() const
        {
            if(this->__net == nullptr)
                throw std::runtime_error("Not attached to network");
            
            return !this->versions_syncronized(*this->__net);
        }

        size_t iterate()
        {
            if(this->__net == nullptr)
                throw std::runtime_error("Not attached to network");

            if(this->__net != nullptr && !this->versions_synchronized(*this->__net))
                    throw version_mismatch(*this, *this->__net);

            assert(this->size() == this->__net->size());
            
            size_t num_changes = 0;
            typename std::map<point<T> , Q>::iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                neighborhood<T, Q> neighbors = this->__net->neighbors(it->first);
                for(size_t i = 0; i < neighbors.size(); i++)
                {
                    assert(this->find(neighbors.at(i)) != this->end());
                    const Q test_cost = it->second + neighbors.at(i).cost;
                    if(test_cost < this->cost(neighbors.at(i)))
                    {
                        this->set(neighbors.at(i), test_cost);
                        num_changes++;
                    }
                }
            }
            return num_changes;
        }

        std::vector<point<T> > overlap(const glnav::network<T, Q> &net) const
        {
            std::vector<point<T> > result;
            typename std::map<point<T> , Q>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                if(net.contains(it->first))
                    result.push_back(it->first);
            }
            return result;
        }

        neighborhood<T, Q> as_neighborhood() const
        {
            neighborhood<T, Q> result;
            typename std::map<point<T> , Q>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                result.push_back(neighbor<T, Q>(it->first, it->second));
            }
            return result;
        }
    private:
        const glnav::network<T, Q> * __net;
    };
}

#endif