#ifndef GLNAV_VERSION_CONTROL_H
#define GLNAV_VERSION_CONTROL_H

#include <inttypes.h>
#include <stdexcept>

namespace glnav
{
    typedef uint64_t version_t;

    class version_controlled
    {
    public:
        version_controlled()
            : __version(0)
        { }

        version_controlled(const version_t &seed)
            : __version(seed)
        { }

        version_controlled(const version_controlled &other)
            : __version(other.__version)
        { }

        void set_version(const version_t version)
        {
            this->__version = version;
        }

        version_t version() const { return this->__version; }
        void update_version() { this->__version++; }
        bool versions_syncronized(const version_controlled &other) const
        { 
            return this->__version == other.__version;
        }
    private:
        version_t __version;
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
}

#endif