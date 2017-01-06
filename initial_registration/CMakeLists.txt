cmake_minimum_required(VERSION 2.8.11)

project (initial_registration)

find_package(PCL 1.8 REQUIRED COMPONENTS common io features search filters keypoints registration visualization)
if ( PCL_FOUND )
	include_directories( ${PCL_INCLUDE_DIRS} )
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions( ${PCL_DEFINITIONS} )
	add_definitions( -DFEVV_USE_PCL )
else()
	message ( "Unfound PCL package")
endif ()

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(initial_registration initial_registration.cpp)

target_link_libraries(initial_registration ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_KEYPOINTS_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})


