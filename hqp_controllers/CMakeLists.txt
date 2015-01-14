cmake_minimum_required(VERSION 2.8.3)
project(hqp_controllers)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
     cmake_modules
     roscpp
#    urdf
#    control_toolbox
     controller_interface
#    controller_manager
#    hardware_interface
#    realtime_tools
#    control_msgs
)

find_package(Eigen REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Declare catkin package
catkin_package(
     CATKIN_DEPENDS
     roscpp
#    urdf
#    control_toolbox
     controller_interface
#    controller_manager
#    hardware_interface
#    realtime_tools
#    control_msgs
     INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
     LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########


# include_directories(include)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/hqp_velocity_controller.cpp
                            include/hqp_controllers/hqp_velocity_controller.h
                            src/task.cpp
                            include/hqp_controllers/task_manager.h
                            src/task_manager.cpp
                            include/hqp_controllers/task.h
                            src/task_object.cpp
                            include/hqp_controllers/task_object.h)

## Declare a cpp executable
# add_executable(hqp_velocity_controller_node src/hqp_velocity_controller_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hqp_velocity_controller_node hqp_velocity_controller_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES hqp_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

