#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<double> midpoint(3, 0);
    const glnav::point<double> target(3, 3);
    const glnav::point<double> start(0, 0);
    glnav::route<double, double> test;
    test.push_back(glnav::heading<double, double>(midpoint, 1));
    test.push_back(glnav::heading<double, double>(target, 1));
    TEST_TRUE(test.target() == target);

    glnav::travel_result<double, double> result = test.follow(start, 1);
    TEST_TRUE(result.target == midpoint);
    TEST_TRUE(result.location.x > 0.99 && result.location.x < 1.01);
    TEST_EQUAL(result.location.y, 0);
    TEST_EQUAL(result.elapsed_time, 1);
    TEST_EQUAL(result.unused_time, 0);
    TEST_EQUAL(result.time_to_waypoint, 3-1);
    
    glnav::point<double> movement = result.location;

    result = test.follow(movement, 3);
    TEST_TRUE(result.target == target);
    TEST_EQUAL(result.location.x, 3);
    TEST_TRUE(result.location.y > 0.99 && result.location.y < 1.01);
    TEST_EQUAL(result.elapsed_time, 3);
    TEST_EQUAL(result.unused_time, 0);
    TEST_EQUAL(result.time_to_waypoint, 3-1);

    movement = result.location;

    result = test.follow(movement, 3);
    TEST_TRUE(result.target == target);
    TEST_TRUE(result.location == target);
    TEST_EQUAL(result.elapsed_time, 3-1);
    TEST_EQUAL(result.unused_time, 3-2);
    TEST_EQUAL(result.time_to_waypoint, 0);
}