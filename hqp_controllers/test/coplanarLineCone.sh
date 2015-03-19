#!/bin/bash

 rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

#clean the controller
rosservice call /apple/apple_hqp_vel_controller/reset_hqp_control 

#load the persistent tasks
rosservice call /apple/apple_hqp_vel_controller/load_tasks "task_definitions"

rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{t_type: 5, priority: 2, is_equality_task: false, task_frame: "world", ds: 0.0, di: 0.3, dynamics: {d_type: 1, d_data: [-1]}, t_links: [{link_frame: "world", geometries: [{g_type: 2, g_data: [1.0, -1.5, 0.14, 0.0, 0.0, 1.0]} ]}, {link_frame: "velvet_fingers_palm", geometries: [{g_type: 8, g_data: [0, 0.0, 0, 1.0, 0.0, 0.0, 0.05]}]} ]} ]}'

rosservice call /apple/apple_hqp_vel_controller/visualize_task_geometries '{ids: [13]}'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true




