#include "glnav.h"
#include "test.h"

int main(void)
{
    const glnav::point<double> point1(0, -2);
    const glnav::point<double> point2(0, 1);
    const glnav::point<double> point3(2, -3);
    const glnav::point<double> point4(2, 2);
    const glnav::point<double> start(-2, 0);
    const glnav::point<double> finish(4, 0);
    glnav::maze<double, double> net(start, finish);
    glnav::path<double> lower(point1, point3);
    glnav::path<double> upper(point2, point4);
    glnav::path<double> start1(start, point1);
    glnav::path<double> start2(start, point2);
    glnav::path<double> finish1(point3, finish);
    glnav::path<double> finish2(point4, finish);
    net.add(lower, lower.length<double>());
    net.add(upper, upper.length<double>());
    net.add(start1, start1.length<double>());
    net.add(start2, start2.length<double>());
    net.add(finish1, start1.length<double>());
    net.add(finish2, start2.length<double>());

    glnav::dijkstra<double, double> test(net);
    TEST_FALSE(test.is_solved());

    const size_t max_iterations = 100;
    size_t iterations = 0;
    while(!test.is_solved() > 0 && iterations++ < max_iterations)
    {
        test.iterate();
    }

    TEST_TRUE(test.is_solved());
    glnav::route<double, double> solution = test.build_route();
    TEST_EQUAL(solution.size(), 3);
    TEST_TRUE(solution[0].target == point2);
    TEST_TRUE(solution[1].target == point4);
    TEST_TRUE(solution[2].target == finish);
}