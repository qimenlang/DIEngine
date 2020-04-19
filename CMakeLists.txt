cmake_minimum_required (VERSION 3.15)

project ("DIEngine")

set(yaml_cpp_dll ${CMAKE_BINARY_DIR}/thirdparty/yaml-cpp)

set(yaml_cpp_dll_file ${CMAKE_BINARY_DIR}/thirdparty/yaml-cpp/yaml-cppd.dll)

set(exe_path ${CMAKE_BINARY_DIR}/engine)

add_subdirectory(thirdparty)

add_subdirectory(obj)

add_subdirectory(engine)


message("CMAKE_SOURCE_DIR:${CMAKE_SOURCE_DIR}")

message("CMAKE_BINARY_DIR:${CMAKE_BINARY_DIR}")

message("yaml_cpp_dll:${yaml_cpp_dll}")

message("exe_path:${exe_path}")

message("EXECUTABLE_OUTPUT_PATH:${EXECUTABLE_OUTPUT_PATH}")



