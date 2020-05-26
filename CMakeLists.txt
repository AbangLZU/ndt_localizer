cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL REQUIRED QUIET)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        pcl_ros
        sensor_msgs
        pcl_conversions
        message_generation
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/voxel_grid_filter.cpp)

add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})

target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

add_executable(map_loader nodes/mapLoader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})