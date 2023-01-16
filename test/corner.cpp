#include "glnav.h"
#include "test.h"
#include <stdio.h>

int main(void)
{
    // Convex case
    {
        const glnav::point<int> center(1, 1);
        const glnav::point<int> leg1(1, 2);
        const glnav::point<int> leg2(0, 1);
        const glnav::point<int> outside(2, 0);
        glnav::corner<int> test(center, leg1, leg2, outside);

        // Starts parallel
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, 1, 3)));
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, -1, 1)));

        // Starts on and points outside
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, 4, 1)));

        // Starts on and points inside
        TEST_TRUE(test.obstructs(glnav::path<int>(1, 1, 0, 2)));

        // Ends on and points outside
        TEST_FALSE(test.obstructs(glnav::path<int>(4, 1, 1, 1)));

        // Ends on and points inside
        TEST_TRUE(test.obstructs(glnav::path<int>(0, 2, 1, 1)));

        // Passes through parallel
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 0, 1, 3)));
        TEST_FALSE(test.obstructs(glnav::path<int>(-1, 1, 2, 1)));

        // Passes through obstructs
        TEST_TRUE(test.obstructs(glnav::path<int>(0, 2, 2, 0)));

        // Passes through but does not obstruct
        TEST_FALSE(test.obstructs(glnav::path<int>(0, 0, 2, 2)));
    }

    // Concave case
    {
        const glnav::point<int> center(1, 1);
        const glnav::point<int> leg1(1, 2);
        const glnav::point<int> leg2(0, 1);
        const glnav::point<int> outside(0, 2);
        glnav::corner<int> test(center, leg1, leg2, outside);

        // Starts parallel
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, 1, 3)));
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, -1, 1)));

        // Starts on and points inside
        TEST_TRUE(test.obstructs(glnav::path<int>(1, 1, 4, 1)));

        // Starts on and points outside
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 1, 0, 2)));

        // Ends on and points inside
        TEST_TRUE(test.obstructs(glnav::path<int>(4, 1, 1, 1)));

        // Ends on and points outside
        TEST_FALSE(test.obstructs(glnav::path<int>(0, 2, 1, 1)));

        // Passes through parallel
        TEST_FALSE(test.obstructs(glnav::path<int>(1, 0, 1, 3)));
        TEST_FALSE(test.obstructs(glnav::path<int>(-1, 1, 2, 1)));

        // Passes through obstructs
        TEST_TRUE(test.obstructs(glnav::path<int>(0, 0, 2, 2)));
    }

    // Comparison
    {
        const glnav::point<int> center(1, 1);
        const glnav::point<int> leg1(1, 3);
        const glnav::point<int> leg2(-1, 1);
        const glnav::point<int> outside(2, 0);
        const glnav::corner<int> test(center, leg1, leg2, outside);

        const glnav::point<int> leg3(1, 2);
        const glnav::point<int> leg4(0, 1);
        const glnav::corner<int> test2(center, leg3, leg4, outside);

        TEST_TRUE(test == test2);

        const glnav::corner<int> test3(center, leg3, leg4, true);
        TEST_TRUE(test == test3);
    }
}