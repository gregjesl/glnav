#ifndef CON3_H
#define CON3_H

#include <set>
#include <vector>
#include <map>

/*! \brief \brief (Con)figuration (Con)trolled (Con)tainers */
namespace con3
{
    /*! \brief Configuration-controlled set 
     *
     * \param T The type of data contained in the set
     */
    template<typename T>
    class set
    {
    public:
        /*! \brief The action to take for the queued value */
        typedef enum action_enum
        {
            ADD,
            REMOVE
        } action_t;

        /*! \brief Default constructor */
        set()
        { 
            this->__reset_cache();
        }

        set(const std::set<T> &other)
            : __values(other)
        {
            this->__reset_cache();
        }

        /*! \brief Copy constructor */
        set(const set &other) 
            : __changes(other.__changes),
            __values(other.__values)
        { 
            this->__reset_cache();
        }

        /*! \brief Assignment operator */
        set& operator=(const set &other)
        {
            this->__changes = other.__changes;
            this->__values = other.__values;
            this->__reset_cache();
            return *this;
        }

        /*! \brief Queues a value to be added 
         *
         * This method will add an add operation to the change queue. The value be added when ::merge or ::synchronize() is called. 
         */
        set<T>& add(T value)
        {
            typename std::map<T, action_t>::iterator it = this->__changes.find(value);
            if(it != this->__changes.end())
            {
                it->second = ADD;
                return *this;
            }
            this->__changes.insert(
                std::pair<T, action_t>(value, ADD)
            );
            return *this;
        }

        /*! \brief Queues a value to be removed 
         *
         * This method will add a remove operation the value to the change queue. The value be removed when ::purge or ::synchronize() is called. 
         */
        set<T>& remove(T value)
        {
            typename std::map<T, action_t>::iterator it = this->__changes.find(value);
            if(it != this->__changes.end())
            {
                it->second = REMOVE;
                return *this;
            }
            this->__changes.insert(
                std::pair<T, action_t>(value, REMOVE)
            );
            return *this;
        }

        /*! \brief Synchronizes the values with the change queue 
         *
         * Upon completion, the change queue will be empty and all queued changes will be made
         * \returns True if changes were made or false if changes where not made (no changes were queued)
         */
        bool synchronize()
        {
            if(this->__changes.empty()) return false;

            while(!this->__changes.empty())
            {
                typename std::map<T, action_t>::iterator it = this->__changes.begin();
                switch (it->second)
                {
                case ADD:
                    this->__values.insert(it->first);
                    break;
                case REMOVE:
                    this->__values.erase(it->first);
                    break;
                }
                this->__changes.erase(it);
            }

            this->__reset_cache();

            return true;
        }

        /*! \brief Synchronizes a specific value in the change queue
         *
         * Upon completion, the change queue will be empty and all queued changes will be made
         */
        bool synchronize(const T value)
        {
            typename std::map<T, action_t>::const_iterator it = this->__changes.find(value);
            if(it == this->__changes.end()) return false;
            switch (it->second)
            {
            case ADD:
                this->__values.insert(it->first);
                break;
            case REMOVE:
                this->__values.erase(it->first);
                break;
            }
            this->__changes.erase(it);
            this->__reset_cache();
            return true;
        }

        /*! \brief Immediately adds a value */
        set<T> & force_add(const T value)
        {
            this->add(value).synchronize(value);
            return *this;
        }

        /*! \brief Immediately removes a value */
        set<T> & force_remove(const T value)
        {
            this->remove(value).synchronize(value);
            return *this;
        }

        /*! \brief Merges all addition to the set of value 
         * 
         * All pending removals remain in the change queue
         * \returns A vector of values that have been added
         */
        std::vector<T> merge()
        {
            std::vector<T> result;
            if(this->__changes.empty()) return result;

            typename std::map<T, action_t>::iterator it = this->__changes.begin();
            while(it != this->__changes.end())
            {
                switch (it->second)
                {
                case ADD:
                    this->__values.insert(it->first);
                    this->__changes.erase(it->first);
                    result.push_back(it->first);
                    it = this->__changes.begin();
                    break;
                case REMOVE:
                    ++it;
                    break;
                }
            }
            this->__reset_cache();
            return result;
        }

        /*! \brief Removes all values pending removal from the set of value 
         * 
         * All pending additions remain in the change queue
         * \returns A vector of values that have been removed
         */
        std::vector<T> purge()
        {
            std::vector<T> result;
            if(this->__changes.empty()) return result;

            typename std::map<T, action_t>::iterator it = this->__changes.begin();
            while(it != this->__changes.end())
            {
                switch (it->second)
                {
                case ADD:
                    ++it;
                    break;
                case REMOVE:
                    this->__values.erase(it->first);
                    this->__changes.erase(it);
                    result.push_back(it->first);
                    it = this->__changes.begin();
                    break;
                }
            }
            this->__reset_cache();
            return result;
        }

        /*! \brief A map of pending changes */
        const std::map<T, action_t> & pending_changes() const { return this->__changes; }

        /*! \brief The current set of values */
        const std::set<T> & values() const { return this->__values; }

        /*! \brief Accesses a specific index in the set 
         *
         * This operator is useful for iterating over the set in a for loop
         * \note This method is O(1) complexity if index increases on each subsequent call
         */
        const T & at(const size_t index)
        {
            if(index >= this->__values.size()) throw std::domain_error("Index exceeds set size");

            if(this->__cached_index < index)
            {
                for(; this->__cached_index < index; this->__cached_index++)
                    ++this->__cached_it;
            }
            else if(this->__cached_index > index)
            {
                assert(this->__cached_index < this->__values.size());
                for(; this->__cached_index < index; this->__cached_index++)
                    --this->__cached_it;
            }
            else
            {
                assert(this->__cached_index == index);
            }

            return *this->__cached_it;
        }
        
        /*! \brief Alias for ::at */
        const T & operator[](const size_t index)
        {
            return this->at(index);
        }
    private:
        std::map<T, action_t> __changes;
        std::set<T> __values;
        size_t __cached_index;
        typename std::set<T>::const_iterator __cached_it;

        void __reset_cache()
        {
            this->__cached_index = 0;
            this->__cached_it = this->__values.begin();
        }
    };
}

#endif