#include "glnav.h"
#include "test.h"
#include <stdio.h>

int main(void)
{
    glnav::cost_map<int> map;

    const glnav::point<int> test_point = glnav::point<int>(1, 1);
    map.seed(test_point);
    TEST_EQUAL(map.size(), 1);
    TEST_EQUAL(map.cost(test_point), std::numeric_limits<double>::infinity());

    map.update(test_point, 11.0);
    TEST_EQUAL(map.cost(test_point), 11.0);
}