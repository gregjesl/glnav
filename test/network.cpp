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

    // Create the network
    glnav::network<int> test_network(&test);
    TEST_EQUAL(test_network.num_nodes(), 8);
    TEST_EQUAL(test_network.num_edges(), 8 + 9);
}