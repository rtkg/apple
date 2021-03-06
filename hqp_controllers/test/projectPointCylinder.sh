#!/bin/bash

 rosservice call /gazebo/set_physics_properties '{time_step: 0.001, max_update_rate: 1000, gravity: [0.0, 0.0, 0.0], ode_config: {auto_disable_bodies: false, sor_pgs_precon_iters: 0, sor_pgs_iters: 50, sor_pgs_w: 1.3, sor_pgs_rms_error_tol: 0.0, contact_surface_layer: 0.001, contact_max_correcting_vel: 100.0, cfm: 0.0, erp: 0.2,  max_contacts: 20} }'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 1, data: [0.0, 0.0, 0.3]}]}, {root: "citi_truck_base", link: "citi_truck_base", geometries: [{type: 9, data: [1.00566, -1.2, 0.25, 0.0, 0.0, 1.0, 0.15]}]} ]}'

rosservice call /apple/apple_hqp_vel_controller/visualize_task_objects '{ids: [8, 9]}'

rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 6, priority: 2, sign: ">=", t_obj_ids: [8, 9], dynamics: {type: 1, data: [-1]}} ]}'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true


