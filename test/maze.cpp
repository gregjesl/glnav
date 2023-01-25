#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<int> corners[4] = 
        {
            glnav::point<int>(-2, 1),
            glnav::point<int>(-2, 5),
            glnav::point<int>(2, 5),
            glnav::point<int>(2, 1)
        };
    glnav::network<int, int> net;
    net.add(glnav::path<int>(corners[0], corners[1]), 1);
    net.add(glnav::path<int>(corners[1], corners[2]), 1);
    net.add(glnav::path<int>(corners[2], corners[3]), 1);
    net.add(glnav::path<int>(corners[3], corners[0]), 1);

    // Build a maze with start and end on the network
    glnav::maze<int, int> test(net, corners[0], corners[2]);
    TEST_EQUAL(test.neighbors(corners[0]).size(), 2);
    TEST_TRUE(test.neighbors(corners[0]).contains(corners[1]));
    TEST_TRUE(test.neighbors(corners[0]).contains(corners[3]));
    TEST_EQUAL(test.neighbors(corners[2]).size(), 2);
    TEST_TRUE(test.neighbors(corners[2]).contains(corners[1]));
    TEST_TRUE(test.neighbors(corners[2]).contains(corners[3]));
    TEST_TRUE(test.is_synchronized());

    // Update the network
    net.add(glnav::path<int>(glnav::point<int>(2, 5), glnav::point<int>(3, 5)), 1);
    net.add(glnav::path<int>(glnav::point<int>(2, 1), glnav::point<int>(3, 1)), 1);
    TEST_FALSE(test.is_synchronized());
    test.update();
    TEST_TRUE(test.is_synchronized());

    // Build a maze with points starting off of the network
}