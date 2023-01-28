#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<double> start(0, 0);
    const glnav::point<double> target(2, 2);
    glnav::heading<double, double> test(target, 1);
    glnav::travel_result<double, double> result = test.approach(start, 1.414);
    TEST_TRUE(result.location.x > 0.99 && result.location.x < 1.01);
    TEST_TRUE(result.location.y > 0.99 && result.location.y < 1.01);
    TEST_TRUE(result.time_to_target > 1.4 && result.time_to_target < 1.5);
    TEST_EQUAL(result.unused_time, 0);
    TEST_EQUAL(result.elapsed_time, 1.414);
    TEST_FALSE(result.target_reached());

    result = test.approach(start, 3);
    TEST_TRUE(result.location == target);
    TEST_TRUE(result.elapsed_time > 2.828 && result.elapsed_time < 2.829);
    TEST_TRUE(result.unused_time < (3-2.828) && result.unused_time > (3 - 2.829));
    TEST_EQUAL(result.time_to_target, 0);
}