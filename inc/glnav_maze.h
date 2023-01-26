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
    class maze : public version_dependent
    {
    public:
        maze(const network<T, Q> &net, 
            const point<T> &start, 
            const point<T> &finish)
            : version_dependent(net),
            __net(net),
            __start(start),
            __finish(finish)
        {
            if(!net.contains(start)) throw unknown_point_exception<T>(start);
            if(!net.contains(finish)) throw unknown_point_exception<T>(finish);
            this->__attach_ramps();
            this->__verify_resync();
        }

        maze(const network<T, Q> &net, 
            const point<T> &start, 
            const point<T> &finish, 
            const cost_map<T, Q> &onramp, 
            const cost_map<T, Q> &offramp)
            : version_dependent(net),
            __net(net),
            __start(start),
            __finish(finish),
            __onramp(onramp),
            __offramp(offramp)
        {
            this->__attach_ramps();
            this->__verify_resync();
        }

        maze(const maze<T, Q> &other)
            : version_dependent(other),
            __net(other.__net),
            __start(other.__start),
            __finish(other.__finish),
            __onramp(other.__onramp),
            __offramp(other.__offramp)
        {
            this->force_synchronization();
            assert(this->__onramp.is_attached());
            assert(this->__offramp.is_attached());
            assert(this->__line_of_sight || this->__onramp.overlap(this->__net).size() > 0);
            assert(this->__line_of_sight || this->__offramp.overlap(this->__net).size() > 0);
        }

        virtual ~maze()
        {
            
        }

        bool is_synchronized() const
        {
            return this->versions_synchronized(this->__net)
                && this->versions_synchronized(this->__onramp)
                && this->versions_synchronized(this->__offramp);
        }

        void update()
        {
            this->__onramp.detatch();
            this->__offramp.detatch();
            this->synchronize_version(this->__net);
            this->__attach_ramps();
            this->__verify_resync();
        }

        void update(const cost_map<T, Q> &onramp, const cost_map<T, Q> &offramp)
        {
            this->__onramp = onramp;
            this->__offramp = offramp;
            this->__attach_ramps();
            if(!this->__net.versions_synchronized(this->__onramp)) throw version_mismatch(this->__net, this->__onramp);
            if(!this->__net.versions_synchronized(this->__offramp)) throw version_mismatch(this->__net, this->__offramp);
            this->set_version(this->__net.version());
            assert(this->is_synchronized());
        }

        void update(const point<T> &start, const point<T> &finish, const cost_map<T, Q> &onramp, const cost_map<T, Q> &offramp)
        {
            this->__start = start;
            this->__finish = finish;
            this->update(onramp, offramp);
            this->__verify_resync();
        }

        neighborhood<T, Q> neighbors(const point<T> &from) const
        {
            this->__force_synchronization();
            
            if(from == this->__start)
            {
                if(this->__onramp.size() > 0)
                {
                    assert(!this->__net.contains(from));
                    return this->__onramp.as_neighborhood();
                }
                assert(this->__net.contains(from));
                return this->__net.neighbors(from);
            }

            if(from == this->__finish)
            {
                if(this->__offramp.size() > 0)
                {
                    assert(!this->__net.contains(from));
                    return this->__offramp.as_neighborhood();
                }
                assert(this->__net.contains(from));
                return this->__net.neighbors(from);
            }

            if(!this->__net.contains(from)) throw unknown_point_exception<T>(from);

            neighborhood<T, Q> result = this->__net.neighbors(from);

            if(this->__onramp.contains(from))
            {
                result.push_back(neighbor<T, Q>(this->__start, this->__onramp.cost(from)));
            }

            if(this->__offramp.contains(from))
            {
                result.push_back(neighbor<T, Q>(this->__finish, this->__offramp.cost(from)));
            }

            return result;
        }

    private:
        const network<T, Q> &__net;
        point<T> __start;
        point<T> __finish;
        cost_map<T, Q> __onramp;
        cost_map<T, Q> __offramp;

        void __verify_resync()
        {
            if(this->__net.contains(this->__start) && this->__onramp.size() > 0) throw std::runtime_error("Starting point on network");
            if(this->__net.contains(this->__finish) && this->__offramp.size() > 0) throw std::runtime_error("Ending point on network");
            this->__force_synchronization();
            if(this->__onramp.overlap(this->__net).size() == 0 && !this->__net.contains(this->__start)) throw std::runtime_error("Onramp does not connect to network");
            if(this->__offramp.overlap(this->__net).size() == 0 && !this->__net.contains(this->__finish)) throw std::runtime_error("Offramp does not connect to network");
        }

        void __attach_ramps()
        {
            if(!this->__onramp.is_attached())
                this->__onramp.attach(this->__net);
            if(!this->__offramp.is_attached())
                this->__offramp.attach(this->__net);
        }

        void __force_synchronization() const
        {
            if(this->version() != this->__net.version()) throw version_mismatch(this->version(), this->__net.version());
            if(this->version() != this->__onramp.version()) throw version_mismatch(this->version(), this->__onramp.version());
            if(this->version() != this->__offramp.version()) throw version_mismatch(this->version(), this->__offramp.version());
            assert(this->is_synchronized());
        }
    };
}

#endif