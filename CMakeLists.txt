cmake_minimum_required(VERSION 2.8.3)
project(io_adc)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rostime roscpp rosconsole roscpp_serialization team_diana_lib)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${catkin_INCLUDE_DIRS} ./include)
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  imu_Adc.msg
  sosp_Adc.msg
  diag_Adc.msg
)

add_service_files(
  FILES
  movingService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

link_directories(/usr/lib)

add_library(common src/utils.cpp)

add_executable(io_adc
  src/io_adc_node.cpp
  src/suspension_reader.cpp
)
target_link_libraries(io_adc ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m common)
add_dependencies(io_adc io_adc_gencpp)
add_dependencies(io_adc common)

install(TARGETS io_adc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
