cmake_minimum_required(VERSION 2.4.6)

project(imu_filter_madgwick)
find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp std_msgs tf class_loader nodelet)
find_package(Boost REQUIRED COMPONENTS system)

# Generate dynamic parameters
generate_dynamic_reconfigure_options(
       cfg/ImuFilterMadgwick.cfg
)

catkin_package(LIBRARIES imu_filter_nodelet)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ./ ./include/ ${dynamic_reconfigure_PACKAGE_PATH})

# create imu_filter_nodelet library
add_library (imu_filter_nodelet src/imu_filter_nodelet.cpp src/imu_filter.cpp)
add_dependencies(imu_filter_nodelet ${PROJECT_NAME}_gencfg)
target_link_libraries (imu_filter_nodelet ${catkin_LIBRARIES} )

# create imu_filter_node executable
message (" \n\n\ ${PROJECT_NAME}_gencfg \n\n")
add_executable(imu_filter_node src/imu_filter_node.cpp)
add_dependencies(imu_filter_node ${PROJECT_NAME}_gencf imu_filter_nodelet)
target_link_libraries (imu_filter_node ${catkin_LIBRARIES} imu_filter_nodelet)
install(TARGETS imu_filter_node
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
