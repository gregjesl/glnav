#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<double> midpoint(3, 0);
    const glnav::point<double> target(3, 3);
    const glnav::point<double> start(0, 0);
    glnav::route<double, double> rte;
    rte.push_back(glnav::heading<double, double>(midpoint, 1));
    rte.push_back(glnav::heading<double, double>(target, 1));
    TEST_TRUE(rte.target() == target);

    glnav::traveler<double, double> test(start, rte);

    bool repeat = true;

    restart:
    glnav::travel_result<double, double> result = test.travel(1);
    TEST_TRUE(result.target == midpoint);
    TEST_TRUE(result.location.x > 0.99 && result.location.x < 1.01);
    TEST_EQUAL(result.location.y, 0);
    TEST_EQUAL(result.elapsed_time, 1);
    TEST_EQUAL(result.unused_time, 0);
    TEST_EQUAL(result.time_to_waypoint, 3-1);
    TEST_TRUE(test.location() == result.location);
    TEST_TRUE(test.is_moving());

    result = test.travel(3);
    TEST_TRUE(result.target == target);
    TEST_EQUAL(result.location.x, 3);
    TEST_TRUE(result.location.y > 0.99 && result.location.y < 1.01);
    TEST_EQUAL(result.elapsed_time, 3);
    TEST_EQUAL(result.unused_time, 0);
    TEST_EQUAL(result.time_to_waypoint, 3-1);
    TEST_TRUE(test.location() == result.location);
    TEST_TRUE(test.is_moving());

    result = test.travel(3);
    TEST_TRUE(result.target == target);
    TEST_TRUE(result.location == target);
    TEST_EQUAL(result.elapsed_time, 3-1);
    TEST_EQUAL(result.unused_time, 3-2);
    TEST_EQUAL(result.time_to_waypoint, 0);
    TEST_TRUE(test.location() == result.location);
    TEST_FALSE(test.is_moving());
    if(repeat)
    {
        test.set(rte);
        test.teleport(start);
        repeat = false;
        goto restart;
    }
}