cmake_minimum_required(VERSION 2.8.3)
project(adc)

find_package(Boost REQUIRED COMPONENTS date_time thread)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation rostime roscpp rosconsole roscpp_serialization)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${catkin_INCLUDE_DIRS} ./)
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

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

link_directories(/usr/lib)


add_executable(adc src/adc.cpp)
target_link_libraries(adc ${catkin_LIBRARIES} ${Boost_LIBRARIES} pci_dask m)
add_dependencies(adc adc_gencpp)
install(TARGETS adc
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
