cmake_minimum_required(VERSION 2.8.11)

project (icp_registration)

find_package(libpointmatcher 1.1.0 REQUIRED)
include_directories("${libpointmatcher_INCLUDE_DIRS}")
message(STATUS "Using libpointmatcher version ${libpointmatcher_VERSION}")

add_executable(icp_registration icp_registration.cpp)
target_link_libraries(icp_registration ${libpointmatcher_LIBRARIES})
