#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<int> test1(1, 1);
    const glnav::point<int> test2(2, 2);
    glnav::point_group<int> test;
    test.push_back(test1);
    test.push_back(test2);
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(test1));
    TEST_TRUE(test.contains(test2));
    TEST_FALSE(test.contains(glnav::point<int>(3, 3)));
}