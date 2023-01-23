#ifndef GLNAV_COST_MAP_H
#define GLNAV_COST_MAP_H

#include "glnav_point.h"
#include "glnav_network.h"
#include <map>

namespace glnav
{
    template<typename T>
    class cost_map : private std::map<point<T> , double>
    {
    public:
        cost_map() 
            : std::map<point<T> , double>(),
            __net(nullptr),
            __seed(0)
        { }

        cost_map(const network<T> &net)
            : std::map<point<T> , double>(net.node_map(std::numeric_limits<double>::infinity())),
            __net(&net),
            __seed(net.version())
        { }

        using std::map<point<T> , double>::size;
        using std::map<point<T> , double>::empty;
        
        double cost(const point<T> &input) const
        {
            // Perform version check
            if(this->__net != nullptr && this->__seed != this->__net->version())
                throw version_mismatch(this->__seed, this->__net->version());

            const typename std::map<point<T> , double>::const_iterator it = this->find(input);
            if(it != this->end())
            {
                return it->second;
            }
            throw std::out_of_range("Point not found");
        }

        void set(const point<T> &input, const double value)
        {
            // Perform version check
            if(this->__net != nullptr)
            {
                if(this->__seed != this->__net->version())
                    throw version_mismatch(this->__seed, this->__net->version());
                
                if(this->__net->contains(input))
                    throw std::out_of_range("Point is not in network");
            }

            const typename std::map<point<T> , double>::iterator it = this->find(input);
            if(it != this->end())
            {
                it->second = value;
            }
            else 
            {
                this->insert(
                    std::pair<point<T>, double>
                        (input, value)
                );
            }
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

        const network<T> network() const { return this->__net; }
        bool is_attached() const { return this->__net != nullptr; }
        bool is_syncronized() const
        {
            if(this->__net == nullptr)
                throw std::runtime_error("Not attached to network");
            
            return this->__seed == this->__net->version();
        }

        size_t iterate()
        {
            if(this->__net == nullptr)
                throw std::runtime_error("Not attached to network");

            if(this->__net->version() != this->__seed)
                throw version_mismatch(this->__seed, this->__net->version());

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
    private:
        const glnav::network<T> * __net;
        version_t __seed;
    };
}

#endif