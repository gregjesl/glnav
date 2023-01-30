#include "glnav.h"
#include "test.h"

int main(void)
{
    glnav::dispatch<double, double> test;
    TEST_EQUAL(test.values().size(), 0);
    const glnav::point<double> start1(1, 1);
    glnav::traveler<double, double> traveler1(start1);
    
    // Attach the traveler
    test.add(&traveler1);
    
    // It hasn't been merged yet
    TEST_EQUAL(test.values().size(), 0);

    // Synchronize
    TEST_TRUE(test.synchronize());
    TEST_EQUAL(test.values().size(), 1);

    // Run for a minute to attach the traveler
    test.run(1);

    // The traveler should not have moved
    TEST_TRUE(traveler1.location() == start1);
}