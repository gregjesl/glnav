#include "glnav.h"
#include "test.h"
#include <stdio.h>

int main(void)
{
    glnav::cost_map<int> map;
    TEST_EQUAL(map.lowest_cost(), std::numeric_limits<double>::infinity());
    TEST_NULL(map.lowest_cost_point());

    const glnav::point<int> test_point = glnav::point<int>(1, 1);
    map.seed(test_point);
    TEST_EQUAL(map.lowest_cost(), std::numeric_limits<double>::infinity());
    TEST_NOT_NULL(map.lowest_cost_point());
    TEST_EQUAL(map.cost(test_point), std::numeric_limits<double>::infinity());

    map.update(test_point, 11.0);
    TEST_NOT_NULL(map.lowest_cost_point());
    TEST_EQUAL(map.lowest_cost(), 11.0);
    TEST_EQUAL(map.cost(test_point), 11.0);

    TEST_EQUAL(map.pointers().size(), 1);
    TEST_NOT_NULL(map.pointers().at(0));
}