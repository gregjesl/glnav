#include "glnav.h"
#include "test.h"

int main(void)
{
    // Create the network
    glnav::network<int> test;
    TEST_EQUAL(test.size(), 0);

    // Add a path
    const glnav::point<int> start(1, 2);
    const glnav::point<int> end(3, 4);
    const glnav::path<int> test_path(start, end);
    test.add(test_path, test_path.length());
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_FALSE(test.contains(glnav::point<int>(5, 6)));
    TEST_TRUE(test.cost(start, end) > 2.0);
    TEST_TRUE(test.cost(start, end) < 4.0);
    TEST_EQUAL(test.neighbors(start).size(), 1);
    TEST_TRUE(test.neighbors(start).at(0).first == end);
    TEST_TRUE(test.neighbors(end).at(0).first == start);

    // Attempt to add the same path
    test.add(test_path, test_path.length());
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_TRUE(test.cost(start, end) > 2.0);
    TEST_TRUE(test.cost(start, end) < 4.0);

    // Node map
    TEST_EQUAL(test.node_map<double>(1.0).size(), 2);
    TEST_TRUE(test.node_map<double>(1.0).begin()->first == start);
    TEST_TRUE(test.node_map<double>(1.0).begin()->second == 1.0);

    // Add a third
    const glnav::point<int> other(6, 5);
    glnav::path<int> test_path2(start, other);
    glnav::path<int> test_path3(end, other);
    test.add(test_path2, test_path2.length());
    test.add(test_path3, test_path3.length());
    TEST_EQUAL(test.size(), 3);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_TRUE(test.contains(other));
    TEST_EQUAL(test.neighbors(start).size(), 2);
    TEST_EQUAL(test.neighbors(end).size(), 2);
    TEST_EQUAL(test.neighbors(other).size(), 2);
    test.remove(other);
    TEST_EQUAL(test.neighbors(start).size(), 1);
    TEST_TRUE(test.neighbors(start).at(0).first == end);
    TEST_TRUE(test.neighbors(end).at(0).first == start);
}