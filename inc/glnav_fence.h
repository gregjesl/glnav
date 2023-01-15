#ifndef GLNAV_FENCE_H
#define GLNAV_FENCE_H

#include "glnav_cartesian.h"
#include "glnav_path.h"
#include <vector>

namespace glnav
{
    template<typename T>
    class fence : virtual public cartesian_object<T>, private std::vector<point<T> *>
    {
    public:
        fence(const point<T> &start, const point<T> &end)
            : std::vector<point<T> *>()
        {
            this->__seed(start, end);
        }

        fence(const path<T> &seed)
            : std::vector<point<T> *>()
        { 
            this->__seed(seed.start, seed.end);
        }

        fence& operator=(const fence &other)
        {
            // Check for same object
            if(this == &other) return *this;

            assert(other.size() > 1);

            for(size_t i = 0; i < this->size(); i++)
            {
                delete this->at(i);
            }
            this->clear();
            this->reserve(other.size());

            for(size_t i = 0; i < other.size(); i++)
            {
                this->push_back(new point<T>(*other.at(i)));
            }
            this->__minX = other.__minX;
            this->__minY = other.__minY;
            this->__maxX = other.__maxX;
            this->__maxY = other.__maxY;
            assert(this->__minX <= this->__maxX);
            assert(this->__minY <= this->__maxY);
            return *this;
        }

        virtual ~fence()
        {
            while(!this->empty())
            {
                delete this->back();
                this->pop_back();
            }
        }

        using std::vector<point<T> *>::size;

        void push_back(const point<T> &input)
        {
            // Do not push the same point to the back
            if(*this->back() == input) return;

            this->push_back(new point<T>(input));
            this->__update_axis(input.x, this->__minX, this->__maxX);
            this->__update_axis(input.y, this->__minY, this->__maxY);
        }

        virtual T minX() const { return this->__minX; }
        virtual T maxX() const { return this->__maxX; }
        virtual T minY() const { return this->__minX; }
        virtual T maxY() const { return this->__maxX; }

        bool is_point() const
        {
            return this->minX() == this->maxX() 
                && this->minY() == this->maxY();
        }

        bool obstructs(const path<T> &input) const
        {
            // Check for fast elimination
            if(!this->could_overlap(input, true)) return false;

            // Iterate through the paths
            for(size_t i = 1; i < this->size(); i++)
            {
                const path<T> segment(*this->at(i-1), *this->at(i));
            }
        }
    private:
        T __minX;
        T __minY;
        T __maxX;
        T __maxY;

        void __seed(const point<T> &start, const point<T> &end)
        {
            this->push_back(new point<T>(start));
            this->push_back(new point<T>(end));
            this->__minX = start.x < end.x ? start.x : end.x;
            this->__maxX = start.x > end.x ? start.x : end.x;
            this->__minY = start.y < end.y ? start.y : end.y;
            this->__maxY = start.y > end.y ? start.y : end.y;
            assert(this->__minX <= this->__maxX);
            assert(this->__minY <= this->__maxY);
        }

        void __update_min(const T input, T &value)
        {
            if(input < value) input = value;
        }

        void __update_max(const T input, T &value)
        {
            if(input > value) input = value;
        }

        void __update_axis(const T value, T &min, T &max)
        {
            this->__update_min(value, min);
            this->__update_max(value, max);
        }
    };
}

#endif