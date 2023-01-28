#ifndef GLNAV_HEADING_H
#define GLNAV_HEADING_H

#include "glnav_point.h"

namespace glnav
{
    template<typename T, typename Q>
    class travel_result
    {
    public:
        point<T> location;
        Q elapsed_time;
        Q unused_time;
        Q time_to_target;

        bool target_reached() const { return this->time_to_target == 0; }
    };

    template<typename T, typename Q>
    class heading
    {
    public:
        heading(const point<T> &target, const Q speed)
            : target(target),
            speed(speed)
        { }

        point<T> target;
        Q speed;

        travel_result<T, Q> approach(const point<T> &location, const Q duration)
        {
            // Initialize result
            travel_result<T, Q> result;

            // Check for on location
            if(location == this->target) {
                result.location = location;
                result.elapsed_time = 0;
                result.unused_time = duration;
                result.time_to_target = 0;
                return result;
            }

            // Get the delta
            const point<T> delta = this->target - location;

            // Get the distance
            const Q distance = glnav::magnitude<T, Q>(delta);

            // Get the time to target
            if(this->speed <= 0) throw std::domain_error("Speed must be greater than zero");
            const Q time_to_target = distance / this->speed;

            if(duration < time_to_target)
            {
                assert(distance > 0);
                const Q traveled = this->speed * duration;
                const point<T> unit(delta.x / distance, delta.y / distance);
                result.location = point<T>(location.x + (unit.x * traveled), location.y + (unit.y * traveled));
                result.elapsed_time = duration;
                result.unused_time = 0;
                result.time_to_target = time_to_target - duration;
                return result;
            }

            assert(duration >= time_to_target);
            result.location = this->target;
            result.elapsed_time = time_to_target;
            result.unused_time = duration - time_to_target;
            result.time_to_target = 0;
            return result;
        }
    };
}

#endif