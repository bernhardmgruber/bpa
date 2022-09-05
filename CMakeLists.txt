cmake_minimum_required(VERSION 3.0)

project(bpa)

# library
find_package(glm CONFIG REQUIRED)
file(GLOB source_files src/lib/*)
add_library(libbpa ${source_files} README.md)
target_link_libraries(libbpa PUBLIC glm::glm)
target_include_directories(libbpa PUBLIC src/lib)
target_compile_features(libbpa PUBLIC cxx_std_20)
if(MSVC)
	target_compile_options(libbpa PRIVATE /W4)
else()
	target_compile_options(libbpa PRIVATE -Wall -Wextra)
endif()
set_target_properties(libbpa PROPERTIES PREFIX "")

# driver
add_executable(bpa src/driver/main.cpp)
target_link_libraries(bpa PUBLIC libbpa)

# tests
include(CTest)
if (BUILD_TESTING)
	find_package(Catch2 3.0.1 REQUIRED)
	file(GLOB test_files test/*)
	add_executable(tests ${test_files})
	source_group("" FILES ${test_files})
	source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${test_files})
	target_link_libraries(tests PRIVATE libbpa Catch2::Catch2WithMain)
	include(Catch)
	catch_discover_tests(tests)
endif()
