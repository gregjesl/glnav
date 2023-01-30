#include "glnav.h"
#include "test.h"

int main(void)
{
    glnav::map<int> test;
    glnav::obstacle<int> test_obs = glnav::obstacle<int>::square(
        glnav::point<int>(1, 1),
        4
    );
    test.force_add(&test_obs);
    TEST_EQUAL(test.values().size(), 1);

    glnav::path<int> test_path(1, -10, 1, 10);
    TEST_TRUE(test.obstructs(test_path));
    test_path = glnav::path<int>(5, -10, 5, 10);
    TEST_FALSE(test.obstructs(test_path));

    std::set<glnav::point<int> > corners = test.corners();
    TEST_EQUAL(corners.size(), 4);
    TEST_TRUE(corners.find(glnav::point<int>(-1, -1)) != corners.end());
    TEST_TRUE(corners.find(glnav::point<int>(-1, 3)) != corners.end());
    TEST_TRUE(corners.find(glnav::point<int>(3, -1)) != corners.end());
    TEST_TRUE(corners.find(glnav::point<int>(3, 3)) != corners.end());
}