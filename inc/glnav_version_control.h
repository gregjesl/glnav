#ifndef GLNAV_VERSION_CONTROL_H
#define GLNAV_VERSION_CONTROL_H

#include <inttypes.h>
#include <stdexcept>

namespace glnav
{
    typedef uint64_t version_t;

    class version_dependent
    {
    public:
        version_dependent()
            : __version(0)
        { }

        version_dependent(const version_t seed)
            : __version(seed)
        { }

        version_dependent(const version_dependent &other)
            : __version(other.__version)
        { }

        void set_version(const version_t version)
        {
            this->__version = version;
        }

        void synchronize_version(const version_dependent &other)
        {
            this->__version = other.__version;
        }

        version_t version() const { return this->__version; }

        bool versions_synchronized(const version_dependent &other) const
        { 
            return this->__version == other.__version;
        }
    private:
        version_t __version;
    };

    class version_controlled : public version_dependent
    {
    public:
        version_controlled()
            : version_dependent()
        { }

        version_controlled(const version_t &seed)
            : version_dependent(seed)
        { }

        version_controlled(const version_dependent &other)
            : version_dependent(other)
        { }

        void update_version() { this->set_version(this->version() + 1); }
    };

    class version_mismatch : public std::runtime_error
    {
    public:
        version_mismatch(const version_t version1, const version_t version2) 
            : std::runtime_error("Version mismatch"),
            expected(version1 < version2 ? version1 : version2),
            actual(version1 > version2 ? version1 : version2)
        { }

        version_mismatch(const version_controlled &version1, const version_controlled &version2) 
            : std::runtime_error("Version mismatch"),
            expected(version1.version() < version2.version() ? version1.version() : version2.version()),
            actual(version1.version() > version2.version() ? version1.version() : version2.version())
        { }

        const version_t expected;
        const version_t actual;
    };

    void assert_version(const version_controlled &version1, const version_controlled &version2)
    {
        if(!version1.versions_synchronized(version2))
            throw version_mismatch(version1, version2);
    }
}

#endif