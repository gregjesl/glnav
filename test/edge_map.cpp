#include "glnav.h"
#include "test.h"

int main(void)
{
    glnav::edge_map<int> test;
    TEST_EQUAL(test.size(), 0);

    const glnav::point<int> test1(1,2);
    const glnav::point<int> test2(3,4);
    test.add(&test1, &test2);
    TEST_EQUAL(test.neighbors(&test1).size(), 1);
    TEST_EQUAL(test.neighbors(&test1).front(), &test2);
    TEST_EQUAL(test.edges(&test1).size(), 1);
}