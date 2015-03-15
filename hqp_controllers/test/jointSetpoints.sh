#!/bin/bash

 rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{t_type: 3, priority: 2, is_equality_task: true, task_frame: "world", ds: 0.01, di: 0.03, dynamics: {d_type: 1, d_data: [-1]}, t_links: [{link_frame: "lbr_iiwa_link_4", geometries: [{g_type: 6, g_data: [1]} ]}, {link_frame: "lbr_iiwa_link_6", geometries: [{g_type: 6, g_data: [-0.5]}]} ]} ]}'


rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true




