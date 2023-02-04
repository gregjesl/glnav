#ifndef GLNAV_MAP_H
#define GLNAV_MAP_H

#include "con3/con3.h"
#include "glnav_path.h"
#include "glnav_obstacle.h"

namespace glnav
{
    template<typename T>
    std::set<point<T> > corners(const std::set<obstacle<T> *> &input)
    {
        std::set<point<T> > result;
        typename std::set<obstacle<T> *>::const_iterator it;
        for(it = input.begin(); it != input.end(); ++it)
        {
            point_group<T> corners = it->corners();
            for(size_t i = 0; i < corners.size(); i++)
                result.insert(corners.at(i));
        }
        return result;
    }

    template<typename T>
    class map : public con3::set<obstacle<T> *>
    {
    public:
        map()
            : con3::set<obstacle<T> *>()
        { }

        map(const std::set<obstacle<T> *> &seed)
            : con3::set<obstacle<T> *>(seed)
        { }

        /*! \brief Finds an obstruction to a path 
         *
         * \returns A pointer to the first obstruction in the set that obstructs the path. `nullptr` is returned if no obstruction is found that obstructs the path. 
         */
        std::vector<obstacle<T> * > obstacles(const path<T> &input)
        {
            std::vector<obstacle<T> * > result;
            typename std::set<obstacle<T> *>::const_iterator it;
            for(it = this->values().begin(); it != this->values().end(); ++it)
            {
                if((*it)->obstructs(input))
                    result.push_back(*it);
            }
            return result;
        }

        std::set<obstacle<T> * > obstacles(const std::set<point<T> > &input)
        {
            std::set<obstacle<T> *> result;
            const std::vector<const path<T> > paths = path<T>::generate(input);
            for(size_t i = 0; i < paths.size(); i++)
            {
                const std::vector<obstacle<T> * > obs = this->obstacles(paths.at(i));
                for(size_t j = 0; j < obs.size(); i++)
                    result.insert(obs.at(i));
            }
            return result;
        }

        bool obstructs(const path<T> &input) const
        {
            typename std::set<obstacle<T> *>::const_iterator it;
            for(it = this->values().begin(); it != this->values().end(); ++it)
            {
                if((*it)->obstructs(input))
                    return true;
            }
            return false;
        }

        std::set<point<T> > corners()
        {
            std::set<point<T> > result;

            // Iterate through the obstacles
            for(size_t i = 0; i < this->values().size(); i++)
            {
                const point_group<T> corners = this->at(i)->corners();

                // Iterate through the corners
                for(size_t j = 0; j < corners.size(); j++)
                {
                    result.insert(corners.at(j));
                }
            }
            return result;
        }

        std::vector<path<T> > paths(const std::set<point<T> > &input) const
        {
            std::vector<path<T> > result;
            typename std::set<point<T> >::const_iterator lower, upper;
            for(lower = input.begin(); lower != input.end(); ++lower)
            {
                for(upper = lower; upper != input.end(); ++upper)
                {
                    if(*lower == *upper) continue;
                    
                    const path<T> test(*lower, *upper);
                    if(!this->obstructs(test))
                        result.push_back(test);
                }
            }
            return result;
        }

        map<T> localize(const cartesian_object<T> &area) const
        {
            std::set<obstacle<T> * > result;
            typename std::set<obstacle<T> *>::const_iterator it;
            for(it = this->values().begin(); it != this->values().end(); ++it)
            {
                if((*it)->could_overlap(area, true))
                    result.insert(*it);
            }
            return map<T>(result);
        }
    };
}

#endif