cmake_minimum_required(VERSION 2.8.3)
project(hqp_controllers)

set(CMAKE_BUILD_TYPE Debug)

# GUROBI libs and includes
set(GUROBI_INCLUDE_DIR "$ENV{GUROBI_HOME}/include") 
set(GUROBI_LIB_DIR "$ENV{GUROBI_HOME}/lib")
set(GUROBI_LIBS gurobi_c++ gurobi60)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
     cmake_modules
     roscpp
     controller_interface
     hqp_controllers_msgs
     kdl_parser
     realtime_tools
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Declare catkin package
catkin_package(
     CATKIN_DEPENDS
     roscpp
     controller_interface
     hqp_controllers_msgs
     kdl_parser
     realtime_tools
     DEPENDS orocos_kdl
     INCLUDE_DIRS include ${EIGEN_INCLUDE_DIRS}
     LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########


# include_directories(include)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
include_directories(${GUROBI_INCLUDE_DIR})

# link_directories
link_directories(${GUROBI_LIB_DIR})

add_library(${PROJECT_NAME} src/hqp_velocity_controller.cpp
                            include/hqp_controllers/hqp_velocity_controller.h
                            src/task.cpp
                            include/hqp_controllers/task.h
                            src/task_dynamics.cpp
                            include/hqp_controllers/task_dynamics.h
                            src/task_manager.cpp
                            include/hqp_controllers/task_manager.h
                            src/hqp.cpp
                            include/hqp_controllers/hqp.h
                            src/task_object.cpp
                            include/hqp_controllers/task_object.h
                            src/task_geometry.cpp
                            include/hqp_controllers/task_geometry.h
                            include/hqp_controllers/conversions.h
                            include/hqp_controllers/utilities.h)

add_executable(hqp_velocity_controller_test test/test.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${GUROBI_LIBS})
target_link_libraries(hqp_velocity_controller_test ${catkin_LIBRARIES})
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(hqp_velocity_controller_node hqp_velocity_controller_generate_messages_cpp)



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

