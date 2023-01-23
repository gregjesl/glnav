#ifndef GLNAV_COST_MAP_H
#define GLNAV_COST_MAP_H

#include "glnav_point.h"
#include "glnav_network.h"
#include "glnav_version_control.h"
#include <map>

namespace glnav
{
    template<typename T>
    class cost_map : private point_map<T, double>,
        public version_dependent
    {
    public:
        cost_map() 
            : point_map<T, double>(),
            version_dependent(),
            __net(nullptr)
        { }

        cost_map(const network<T> &net)
            : point_map<T, double>(net.node_map(std::numeric_limits<double>::infinity())),
            version_dependent(net),
            __net(&net)
        { }

        using point_map<T, double>::size;
        using point_map<T, double>::empty;
        
        double cost(const point<T> &input) const
        {
            // Perform version check
            if(this->__net != nullptr && !this->versions_synchronized(*this->__net))
                throw version_mismatch(*this, *this->__net);
            assert(this->contains(input));
            return this->force_get(input);
        }

        bool set(const point<T> &input, const double value)
        {
            // Perform version check
            if(this->__net != nullptr)
            {
                if(this->__net != nullptr && !this->versions_synchronized(*this->__net))
                    throw version_mismatch(*this, *this->__net);
                
                if(!this->__net->contains(input))
                    throw unknown_point_exception<T>(input);
            }

            return point_map<T, double>::set(input, value); // Returns true if new
        }

        void attach(const glnav::network<T> &net)
        {
            if(this->__net != nullptr)
                throw std::runtime_error("Already attached to a network");
            
            if(!this->empty())
            {
                typename std::map<point<T> , double>::const_iterator it;
                for(it = this->begin(); it != this->end(); ++it)
                {
                    if(!net.contains(it->first))
                        throw std::runtime_error("Orphaned point");
                }
            }

            this->__net = &net;
            this->__seed = net.version();
        }

        void detatch()
        {
            this->__net = nullptr;
            this->__seed = 0;
        }

        const network<T>& network() const { return this->__net; }
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
            typename std::map<point<T> , double>::iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                std::vector<std::pair<point<T>, double> > neighbors = this->__net->neighbors(it->first);
                for(size_t i = 0; i < neighbors.size(); i++)
                {
                    assert(this->find(neighbors.at(i).first) != this->end());
                    const double test_cost = it->second + neighbors.at(i).second;
                    if(test_cost < this->cost(neighbors.at(i).first))
                    {
                        this->set(neighbors.at(i).first, test_cost);
                        num_changes++;
                    }
                }
            }
            return num_changes;
        }

        std::vector<point<T> > overlap(const glnav::network<T> &net) const
        {
            std::vector<point<T> > result;
            typename std::map<point<T> , double>::const_iterator it;
            for(it = this->begin(); it != this->end(); ++it)
            {
                if(net.contains(it->first))
                    result.push_back(it->first);
            }
            return result;
        }

        version_t version() const { return this->__seed; }
    private:
        const glnav::network<T> * __net;
    };
}

#endif