include_directories(../)

file(GLOB tests
    "*.cpp"
)

foreach(test ${tests})
    string(REGEX REPLACE ".*/" "" test_name "${test}")
    string(REGEX REPLACE ".cpp$" "" test_name "${test_name}")
    add_executable ("${test_name}_test" ${test})
    include_directories("${test_name}_test" ..)
	if(MSVC)
		set_property(TARGET "${test_name}_test" PROPERTY _CRT_SECURE_NO_WARNINGS)
	endif()
    add_test(NAME ${test_name} COMMAND "${test_name}_test")
endforeach()