#include "glnav.h"
#include <iostream>
#include <fstream>

int main(void)
{
    std::ofstream output;
    output.open("obstacle.html");
    if(!output.is_open())
    {
        std::cerr << "Could not open file!\n";
        exit(1);
    }

    output << "<!DOCTYPE html>\n";
    output << "<html>\n";
    output << "<body>\n";
    output << "<svg>\n";

    for(size_t i = 3; i < 9; i++)
    {
        output << glnav::obstacle<int>::regular_polygon(
            glnav::point<int>((i-2) * 40, 50),
            15,
            M_PI_4,
            i
        ).svg();
    }

    output << "</svg>\n";
    output << "</html>\n";
    output << "</body>\n";
    
    output.close();
}