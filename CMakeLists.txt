cmake_minimum_required(VERSION 3.0)

project(bpa)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find packages
find_package(glm CONFIG REQUIRED)
find_package(GTest MODULE REQUIRED)

# library
file(GLOB source_files src/*)
add_library(${PROJECT_NAME} ${source_files} README.md)
source_group("" FILES ${source_files})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${source_files})

target_link_libraries(${PROJECT_NAME} PRIVATE
	glm
)

if(MSVC)
	target_compile_options(${PROJECT_NAME} PRIVATE /W4 /experimental:module)
else()
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

# tests
file(GLOB test_files test/*)
add_executable(tests ${test_files})
source_group("" FILES ${test_files})
source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${test_files})

target_link_libraries(tests PRIVATE
	${PROJECT_NAME}
	GTest::GTest GTest::Main
	glm
)
