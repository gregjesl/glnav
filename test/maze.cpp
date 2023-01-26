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
    const glnav::point<int> ur(3, 5);
    const glnav::point<int> lr(3, 1);
    net.add(glnav::path<int>(corners[2], ur), 1);
    net.add(glnav::path<int>(corners[3], lr), 1);
    TEST_FALSE(test.is_synchronized());
    test.update();
    TEST_TRUE(test.is_synchronized());

    // Build a maze with points starting off of the network
    const glnav::point<int> start(-4, 0);
    glnav::cost_map<int, int> onramp;
    onramp.set(corners[0], 2);
    onramp.set(corners[1], 2);
    const glnav::point<int> finish(4, 0);
    glnav::cost_map<int, int> offramp;
    offramp.set(ur, 2);
    offramp.set(lr, 2);
    test.update(start, finish, onramp, offramp);
    TEST_EQUAL(test.neighbors(start).size(), 2);
    TEST_TRUE(test.neighbors(start).contains(corners[0]));
    TEST_TRUE(test.neighbors(start).contains(corners[1]));
    TEST_EQUAL(test.neighbors(finish).size(), 2);
    TEST_TRUE(test.neighbors(finish).contains(ur));
    TEST_TRUE(test.neighbors(finish).contains(lr));
    TEST_EQUAL(test.neighbors(corners[0]).size(), 3);
    TEST_TRUE(test.neighbors(corners[0]).contains(start));
    TEST_EQUAL(test.neighbors(lr).size(), 2);
    TEST_TRUE(test.neighbors(lr).contains(finish));
}