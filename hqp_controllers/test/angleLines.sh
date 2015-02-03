#!/bin/bash

rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "citi_truck_base", link: "citi_truck_base", geometries: [{type: 8, data: [1.25, -1.25, 0.8, 0.0, 0.0, 1.0, 0.7854]}]}, {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 2, data: [0.0, 0.0, 0.0, 0.0, -1.0, 0.0]}]}  ]}'

rosservice call /apple/apple_hqp_vel_controller/visualize_task_objects '{ids: [0, 1]}'

rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 5, priority: 1, sign: "<=", t_obj_ids: [0, 1], dynamics: {type: 1, data: [-1.0]}} ]}'



