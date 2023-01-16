#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::fixed_point<int> test1(1, 1);
    const glnav::fixed_point<int> test2(2, 2);
    glnav::point_group<int> test;
    test.insert(test1);
    test.insert(test2);
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(*test.begin() == test1);
    TEST_TRUE(test.contains(test1));
    TEST_TRUE(test.contains(test2));
    TEST_FALSE(test.contains(glnav::fixed_point<int>(3, 3)));
    glnav::point_group<int> test_translated = test.translate(-3, -3);
    TEST_EQUAL(test_translated.size(), 2);
    TEST_TRUE(*test_translated.begin() == glnav::fixed_point<int>(-1, -1));
}