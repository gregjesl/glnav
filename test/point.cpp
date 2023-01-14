#include "glnav.h"
#include "test.h"

typedef glnav::point<int> test_point;

int main(void)
{
    test_point test(1, 2);
    TEST_EQUAL(test.x, 1);
    TEST_EQUAL(test.y, 2);

    {
        test_point test2(test);
        TEST_EQUAL(test2.x, 1);
        TEST_EQUAL(test2.y, 2);

        test += test2;
        TEST_EQUAL(test.x, 2);
        TEST_EQUAL(test.y, 4);

        test -= test2;
        TEST_EQUAL(test.x, 1);
        TEST_EQUAL(test.y, 2);

        test_point test3 = test + test2;
        TEST_EQUAL(test3.x, 2);
        TEST_EQUAL(test3.y, 4);

        test_point test4 = test3 - test;
        TEST_EQUAL(test4.x, 1);
        TEST_EQUAL(test4.y, 2);

        TEST_TRUE(test == test2);
        TEST_FALSE(test != test2);

        TEST_EQUAL(test.dot(test2), 5);
    }

    TEST_EQUAL(test.magnitude_squared(), 5);
    
}