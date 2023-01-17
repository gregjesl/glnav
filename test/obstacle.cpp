#include "glnav.h"
#include "test.h"

int main(void)
{
    // Create an obstacle
    glnav::obstacle<int> test_obs = glnav::obstacle<int>::square(
        glnav::point<int>(1, 1), 2
    );

    // Test the bounds
    TEST_EQUAL(test_obs.minX(), 0);
    TEST_EQUAL(test_obs.maxX(), 2);
    TEST_EQUAL(test_obs.minY(), 0);
    TEST_EQUAL(test_obs.maxY(), 2);

    // Test the corners
    glnav::point_group<int> corners = test_obs.corners();
    TEST_EQUAL(corners.size(), 4);
    TEST_TRUE(corners.contains(glnav::point<int>(0, 0)));
    TEST_TRUE(corners.contains(glnav::point<int>(0, 2)));
    TEST_TRUE(corners.contains(glnav::point<int>(2, 0)));
    TEST_TRUE(corners.contains(glnav::point<int>(2, 2)));

    // Test obstruction via wall
    glnav::path<int> test_path(-1, 1, 3, 1);
    TEST_TRUE(test_obs.obstructs(test_path));

    // Test obstruction via corner
    test_path = glnav::path<int>(-1, -1, 3, 3);
    TEST_TRUE(test_obs.obstructs(test_path));

    // Test overlapping wall
    test_path = glnav::path<int>(-1, 0, 3, 0);
    TEST_FALSE(test_obs.obstructs(test_path));

    // Test along wall
    test_path = glnav::path<int>(0, 2, 2, 2);
    TEST_FALSE(test_obs.obstructs(test_path));

    // Test outside
    test_path = glnav::path<int>(-1, -1, 3, -1);
    TEST_FALSE(test_obs.obstructs(test_path));
}