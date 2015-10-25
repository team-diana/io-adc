cmake_minimum_required(VERSION 2.8.3)
project(io_adc)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++1y" COMPILER_SUPPORTS_CXX1Y)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX1Y)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++1y")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(PythonInterp)
find_package(PythonLibs REQUIRED)

find_package(Boost REQUIRED COMPONENTS system date_time thread python)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs rostime roscpp rosconsole roscpp_serialization team_diana_lib)

include_directories(${catkin_INCLUDE_DIRS} ./include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
  FILES
  imu_Adc.msg
  sosp_Adc.msg
  diag_Adc.msg
  io_scope.msg
  wheel_motor_temperature.msg
)

add_service_files(
  FILES
  OnOffService.srv
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


install(TARGETS common
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

target_link_libraries(common ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m common ${PYTHON_LIBRARY})

add_executable(io_adc
  src/io_adc_node.cpp
  src/suspension_reader.cpp
  src/motor_temp_sensor_reader.cpp
  src/current_reader.cpp
)

# NOTE: ${PYTHON_LIBRARY} is not need for the executable, but it avoids meaningless linker errors.
target_link_libraries(io_adc ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m common ${PYTHON_LIBRARY})
add_dependencies(io_adc io_adc_gencpp)
add_dependencies(io_adc common)

install(TARGETS io_adc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(adc_scope
  src/adc_scope_node.cpp
)
target_link_libraries(adc_scope ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m common ${PYTHON_LIBRARY})
add_dependencies(adc_scope io_adc_gencpp)
add_dependencies(adc_scope common)

install(TARGETS adc_scope
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(io_scope
  src/io_scope_node.cpp
)
target_link_libraries(io_scope ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m common ${PYTHON_LIBRARY})
add_dependencies(io_scope io_adc_gencpp)
add_dependencies(io_scope common)


install(TARGETS io_scope
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(io_card_python SHARED src/io_card_python.cpp)
target_link_libraries(io_card_python ${Boost_LIBRARIES} ${PYTHON_LIBRARY} pci_dask)
set_target_properties(io_card_python PROPERTIES PREFIX "" )

install(TARGETS io_card_python
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_library(adc_card_python SHARED src/adc_card_python.cpp)
target_link_libraries(adc_card_python ${Boost_LIBRARIES} ${PYTHON_LIBRARY} pci_dask)
set_target_properties(adc_card_python PROPERTIES PREFIX "" )

install(TARGETS adc_card_python
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
