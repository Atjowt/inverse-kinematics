cmake_minimum_required(VERSION 3.5.0)
project(invkin VERSION 0.1.0 LANGUAGES C)
set(CMAKE_C_STANDARD 99)

include(CTest)
enable_testing()

add_executable(${PROJECT_NAME} src/glad.c src/main.c)
target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} PRIVATE m GL glfw dl cglm)
add_subdirectory(cglm/)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
