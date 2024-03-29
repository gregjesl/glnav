#include "glnav.h"
#include "test.h"

int main(void)
{
    // Create the network
    glnav::network<int, double> test;
    TEST_EQUAL(test.size(), 0);
    TEST_EQUAL(test.version(), 0);

    // Add a path
    const glnav::point<int> start(1, 2);
    const glnav::point<int> end(3, 4);
    const glnav::path<int> test_path(start, end);
    test.add(test_path, test_path.length<double>());
    TEST_EQUAL(test.version(), 1);
    TEST_EQUAL(test.size(), 2);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_TRUE(test.contains(test_path));
    TEST_FALSE(test.contains(glnav::point<int>(5, 6)));
    TEST_EQUAL(test.neighbors(start).size(), 1);
    TEST_TRUE(test.neighbors(start).at(0).location() == end);
    TEST_TRUE(test.neighbors(end).at(0).location() == start);
    TEST_TRUE(test.neighbors(start).at(0).cost == test_path.length<double>());
    TEST_TRUE(test.neighbors(end).at(0).cost == test_path.length<double>());

    // Node map
    TEST_EQUAL(test.node_map(1.0).size(), 2);
    TEST_TRUE(test.node_map(1.0).begin()->first == start);
    TEST_TRUE(test.node_map(1.0).begin()->second == 1.0);

    // Add a third
    const glnav::point<int> other(6, 5);
    glnav::path<int> test_path2(start, other);
    glnav::path<int> test_path3(end, other);
    test.add(test_path2, test_path2.length<double>());
    test.add(test_path3, test_path3.length<double>());
    TEST_EQUAL(test.version(), 3);
    TEST_EQUAL(test.size(), 3);
    TEST_TRUE(test.contains(start));
    TEST_TRUE(test.contains(end));
    TEST_TRUE(test.contains(other));
    TEST_EQUAL(test.neighbors(start).size(), 2);
    TEST_EQUAL(test.neighbors(end).size(), 2);
    TEST_EQUAL(test.neighbors(other).size(), 2);
    test.remove(other);
    TEST_EQUAL(test.version(), 4);
    TEST_FALSE(test.contains(other));
    TEST_FALSE(test.contains(test_path2));
    TEST_FALSE(test.contains(test_path3));
    TEST_EQUAL(test.neighbors(start).size(), 1);
    TEST_TRUE(test.neighbors(start).at(0).location() == end);
    TEST_TRUE(test.neighbors(end).at(0).location() == start);
    TEST_TRUE(test.neighbors(start).at(0).cost == test_path.length<double>());
    TEST_TRUE(test.neighbors(end).at(0).cost == test_path.length<double>());

    // Connect networks
    glnav::network<int, double> next;
    next.add(glnav::path<int>(end, other), 1.0);
    TEST_EQUAL(test.overlap(next).size(), 1);
    TEST_TRUE(test.overlap(next).at(0) == end);

    // Copy network
    glnav::network<int, double> construct(test);
    TEST_TRUE(construct.contains(start));
    TEST_TRUE(construct.contains(end));
}