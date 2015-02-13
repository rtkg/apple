cmake_minimum_required(VERSION 2.8.3)
project(hqp_controllers_msgs)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  std_msgs
  visualization_msgs
  geometry_msgs)

#catkin_python_setup()

# Add message and service files
add_message_files(
  FILES
  Task.msg
  TaskObject.msg
  TaskGeometry.msg
  TaskDynamics.msg
  TaskStatus.msg
  TaskStatuses.msg
  )

add_service_files(
  FILES
  SetTasks.srv
  SetTaskObjects.srv
  VisualizeTaskObjects.srv
  ActivateHQPControl.srv
  RemoveTasks.srv
  RemoveTaskObjects.srv
  FindCanTask.srv
  )


# add_dependencies(lbr_fri_node lbr_fri_generate_messages_cpp)
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  std_msgs
  geometry_msgs
  visualization_msgs
  )

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs 
  visualization_msgs
  geometry_msgs
  )

