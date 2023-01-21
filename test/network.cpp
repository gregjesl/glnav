#include "glnav.h"
#include "test.h"

int main(void)
{
    // Create the network
    glnav::network<int, double> test;
    TEST_EQUAL(test.size(), 0);

    // Add a path
    const glnav::point<int> start(1, 2);
    const glnav::point<int> end(3, 4);
    const glnav::path<int> test_path(start, end);
    test.add(test_path, test_path.length(), 1.0);
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_FALSE(test.contains(glnav::point<int>(5, 6)));
    TEST_TRUE(test.cost(start, end) > 2.0);
    TEST_TRUE(test.cost(start, end) < 4.0);
    TEST_EQUAL(test.neighbors(start).size(), 1);
    TEST_TRUE(test.neighbors(start).at(0).first == end);
    TEST_TRUE(test.neighbors(end).at(0).first == start);
    TEST_NOT_NULL(test.metadata(start));
    TEST_EQUAL(*test.metadata(start), 1.0);
    TEST_NULL(test.metadata(glnav::point<int>(5, 6)));
    *test.metadata(start) = 2;
    TEST_EQUAL(*test.metadata(start), 2.0);

    // Attempt to add the same path
    test.add(test_path, test_path.length(), 1.0);
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_TRUE(test.cost(start, end) > 2.0);
    TEST_TRUE(test.cost(start, end) < 4.0);
}