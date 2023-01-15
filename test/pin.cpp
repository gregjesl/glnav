#include "glnav.h"
#include "test.h"
#include <stdio.h>

struct test_case
{
    int x1;
    int y1;
    int x2;
    int y2;
    int x3;
    int y3;
    int x4;
    int y4;
    int x5;
    int y5;
    bool should_cross;
};

test_case test_cases[] = 
{
    { 1, 1, 2, 2, 2, 0, 2, 1, 0, 1, true },
    { 1, 1, 2, 2, 2, 0, 3, 2, 3, 0, false },
    { 0, 0, 0, 1, 0, -1, 1, 0, -1, 0, true }
};

typedef glnav::point<int> test_point;

int main(void)
{
    const size_t num_cases = sizeof(test_cases) / sizeof(test_case);

    for(size_t i = 0; i < num_cases; i++)
    {
        printf("Test case %lu...\n", i);
        const test_case& test = test_cases[i];
        const glnav::pin<int> test_pin(test.x1, test.y1, test.x2, test.y2, test.x3, test.y3);
        TEST_EQUAL(test_pin.blocks(test_point(test.x4, test.y4), test_point(test.x5, test.y5)), test.should_cross);
    }
}