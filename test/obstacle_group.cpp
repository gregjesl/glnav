#include "glnav.h"
#include "test.h"

int main(void)
{
    // Create two obstacles
    glnav::obstacle<int> test_obs1 = glnav::obstacle<int>::square(
        glnav::point<int>(2, 2), 2
    );

    glnav::obstacle<int> test_obs2 = glnav::obstacle<int>::square(
        glnav::point<int>(-2, -2), 2
    );

    // Create the group
    glnav::obstacle_group<int> test;
    test.add(&test_obs1);
    test.add(&test_obs2);
    TEST_EQUAL(test.size(), 2);

    // Check for obstructions
    glnav::path<int> test_path(-10, 2, 10, 2);
    TEST_TRUE(test.obstructs(test_path));
    
    // Check corners
    TEST_EQUAL(test.corners().size(), 8);
}