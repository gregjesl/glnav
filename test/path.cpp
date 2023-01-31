#include "glnav.h"
#include "test.h"
#include <stdio.h>

struct test_case
{
    int x1;
    int y1;
    int x2;
    int y2;
    int x3;
    int y3;
    int x4;
    int y4;
    bool can_overlap;
    bool should_overlap;
};

test_case test_cases[] = 
{
    { -2, -2, 0, 2, 0, 0, 2, 0, false, false }, // Does not intersect
    { -2, -2, 0, 2, 0, 0, 2, 0, true, false }, // Does not intersect
    { 1, 2, 3, 4, 1, 2, -3, -4, true, false }, // Start overlaps with start
    { 1, 2, 3, 4, 1, 2, -3, -4, false, true }, // Start overlaps with start
    { 1, 2, 3, 4, -5, -6, 1, 2, true, false }, // Start overlaps with end
    { 1, 2, 3, 4, -5, -6, 1, 2, false, true }, // Start overlaps with end
    { 1, 2, 3, 4, 3, 4, 7, 8, true, false }, // End overlaps with start
    { 1, 2, 3, 4, 3, 4, 7, 8, false, true }, // End overlaps with start
    { 1, 2, 3, 4, 5, 6, 3, 4, true, false }, // End overlaps with end
    { 1, 2, 3, 4, 5, 6, 3, 4, false, true }, // End overlaps with end
    { 1, 2, 3, 4, -2, 2, -1, 4, false, false }, // Completely left
    { 1, 2, 3, 4, -2, 2, -1, 4, true, false }, // Completely left
    { 1, 2, 3, 4, 5, 2, 7, 4, false, false }, // Completely right
    { 1, 2, 3, 4, 5, 2, 7, 4, true, false }, // Completely right
    { 1, 2, 3, 4, 1, 6, 3, 8, false, false }, // Completely above
    { 1, 2, 3, 4, 1, 6, 3, 8, true, false }, // Completely above
    { 1, 2, 3, 4, 1, -2, 3, -1, false, false }, // Completely below
    { 1, 2, 3, 4, 1, -2, 3, -1, true, false }, // Completely below
    { -5, -5, 5, 5, 1, 2, 2, 3, false, false }, // Close but not touching
    { -5, -5, 5, 5, -1, -2, 2, 1, false, false }, // Close but not touching
    { -5, -5, 5, 5, -6, -6, -4, -4, false, true }, // Overlapping
    { -5, -5, 5, 5, -6, -6, 6, 6, false, true }, // Overlapping
    { -5, -5, 5, 5, -4, -4, 4, 4, false, true }, // Overlapping
    { -5, -5, 5, 5, -5, 5, 5, -5, false, true }, // Crossing
    { -5, -5, 5, 5, 0, 0, 5, -5, true, false }, // Starts on
    { -5, -5, 5, 5, 0, 0, 5, -5, false, true }, // Starts on
    { -5, -5, 5, 5, -5, 5, 0, 0, true, false }, // Ends on
    { -5, -5, 5, 5, -5, 5, 0, 0, false, true }, // Ends on
    { 0, 0, 5, 5, -5, 5, 5, -5, true, false }, // Starts on
    { 0, 0, 5, 5, -5, 5, 5, -5, false, true }, // Starts on
    { -5, -5, 0, 0, -5, 5, 5, -5, true, false }, // Ends on
    { -5, -5, 0, 0, -5, 5, 5, -5, false, true } // Ends on
};

int main(void)
{
    const size_t num_cases = sizeof(test_cases) / sizeof(test_case);

    for(size_t i = 0; i < num_cases; i++)
    {
        printf("Test case %lu...\n", i);
        const test_case& test = test_cases[i];
        const glnav::path<int> path1(test.x1, test.y1, test.x2, test.y2);
        const glnav::path<int> path2(test.x3, test.y3, test.x4, test.y4);
        TEST_EQUAL(path1.intersects(path2, test.can_overlap), test.should_overlap);
    }

    // Comparison
    {
        TEST_TRUE(
            glnav::path<int>(0, 0, 1, 1) <
            glnav::path<int>(0, 0, 2, 2)
        );

        TEST_TRUE(
            glnav::path<int>(0, 0, 1, 1) <
            glnav::path<int>(1, 1, 2, 2)
        );

        TEST_TRUE(
            glnav::path<int>(0, 0, 0, 2) <
            glnav::path<int>(0, 1, 2, 1)
        );

        TEST_FALSE(
            glnav::path<int>(0, 0, 0, 2) <
            glnav::path<int>(0, 0, 0, 2)
        );

        TEST_TRUE(
            glnav::path<int>(0, 0, 0, 2) ==
            glnav::path<int>(0, 2, 0, 0)
        );
    }

    // Generation
    {
        std::set<glnav::point<int> > input;
        input.insert(glnav::point<int>(0, 1));
        input.insert(glnav::point<int>(-1, -1));
        input.insert(glnav::point<int>(1, -1));
        const std::vector<glnav::path<int> > paths = glnav::path<int>::generate(input);
        TEST_EQUAL(paths.size(), 3);
        TEST_CONTAINS(paths, glnav::path<int>(glnav::point<int>(0, 1), glnav::point<int>(-1, -1)));
        TEST_CONTAINS(paths, glnav::path<int>(glnav::point<int>(1, -1), glnav::point<int>(-1, -1)));
        TEST_CONTAINS(paths, glnav::path<int>(glnav::point<int>(0, 1), glnav::point<int>(1, -1)));
    }
}