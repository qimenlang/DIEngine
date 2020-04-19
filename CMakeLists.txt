cmake_minimum_required (VERSION 3.8)

project ("DIEngine")

add_executable (DIEngine "DIEngine.cpp" "DIEngine.h")

add_subdirectory(thirdparty)

target_link_libraries(DIEngine PUBLIC glfw)