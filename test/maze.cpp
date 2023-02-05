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
    glnav::maze<int, int> test(corners[0], corners[2]);
    TEST_FALSE(test.is_valid());
    test.add(glnav::path<int>(corners[0], corners[1]), 1);
    test.add(glnav::path<int>(corners[1], corners[2]), 1);
    test.add(glnav::path<int>(corners[2], corners[3]), 1);
    test.add(glnav::path<int>(corners[3], corners[0]), 1);
    TEST_TRUE(test.is_valid());
}