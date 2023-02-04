cmake_minimum_required (VERSION 3.8)

project(glnav VERSION 0.0.1)

if(MSVC)
    # ignore warnings about scanf
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_compile_options(/W4 /WX)
elseif(BORLAND)
else()
    add_compile_options(-Wall -Wextra -Werror)
endif()

include_directories(lib)

enable_testing()
add_subdirectory (test)
add_subdirectory (demo)