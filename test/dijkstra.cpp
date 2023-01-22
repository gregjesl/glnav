#include "glnav.h"
#include "test.h"

int main(void)
{
    glnav::network<double> net;
    const glnav::point<double> point1(0, -2);
    const glnav::point<double> point2(0, 1);
    const glnav::point<double> point3(2, -3);
    const glnav::point<double> point4(2, 2);
    const glnav::point<double> start(-2, 0);
    const glnav::point<double> finish(4, 0);
    glnav::path<double> lower(point1, point3);
    glnav::path<double> upper(point2, point4);
    glnav::path<double> start1(start, point1);
    glnav::path<double> start2(start, point2);
    glnav::path<double> finish1(point3, finish);
    glnav::path<double> finish2(point4, finish);
    net.add(lower, lower.length());
    net.add(upper, upper.length());
    net.add(start1, start1.length());
    net.add(start2, start2.length());
    net.add(finish1, start1.length());
    net.add(finish2, start2.length());

    glnav::dijkstra<double> test(net, start, finish);
    TEST_FALSE(test.is_solved());

    while(test.iterate() > 0)
    {
        // Loop
    }

    TEST_TRUE(test.is_solved());
    const glnav::point_group<double> solution = test.route();
    TEST_EQUAL(solution.size(), 4);
    TEST_TRUE(solution.at(0) == start);
}