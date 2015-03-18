rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /apple/apple_hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /apple/apple_hqp_vel_controller/load_tasks "task_definitions"

#set the object transfer tasks 
rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks:
[{t_type: 1, priority: 2, is_equality_task: true, task_frame: "world", ds: 0.0, di: 0.05, dynamics:
{d_type: 1, d_data: [-0.5]}, t_links: [{link_frame: "world", geometries: [{g_type: 3, g_data: [0, 0,
1, 0.5]}]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 1, g_data: [0, 0, 0]}]}]}, {t_type: 2, priority: 2, is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.1,
dynamics: {d_type: 1, d_data: [-5]}, t_links: [{link_frame: "world", geometries: [{g_type: 8,
g_data: [0, 0, 0, 0, 0, 1, 0.0]}]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 2,
g_data: [0, 0, 0, 0, 0, 1]}]}]} ]}'

#visualize the tasks
rosservice call /apple/apple_hqp_vel_controller/visualize_task_geometries '{ids: [7, 8, 9, 10, 11, 12, 13, 14]}'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true
