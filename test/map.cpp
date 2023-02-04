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

    glnav::cartesian_area<int> area(glnav::point<int>(0, 0));
    area.expand(glnav::point<int>(-2, -2));
    glnav::map<int> local = test.localize(area);
    TEST_EQUAL(local.values().size(), 1);
    TEST_EQUAL(*local.values().begin(), &test_obs);

    area = glnav::cartesian_area<int>(glnav::point<int>(-2, -2));
    area.expand(glnav::point<int>(-4, -4));
    local = test.localize(area);
    TEST_EQUAL(local.values().size(), 0);
}