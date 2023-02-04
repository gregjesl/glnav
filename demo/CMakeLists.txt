include_directories(../)

file(GLOB tests
    "*.cpp"
)

foreach(test ${tests})
    string(REGEX REPLACE ".*/" "" test_name "${test}")
    string(REGEX REPLACE ".cpp$" "" test_name "${test_name}")
    add_executable ("${test_name}" ${test})
    include_directories("${test_name}" ..)
	if(MSVC)
		set_property(TARGET "${test_name}" PROPERTY _CRT_SECURE_NO_WARNINGS)
	endif()
endforeach()