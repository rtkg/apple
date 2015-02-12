#!/bin/bash

./resetController.sh

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control false

#set the joint limit objects
rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "lbr_iiwa_link_0", link: "lbr_iiwa_link_1", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.9671, -2.8798, -1.7453, 2.9671, 2.8798, 1.7453]}]}, {root: "lbr_iiwa_link_1", link: "lbr_iiwa_link_2", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.0944, -2.0071, -0.8727, 2.0944, 2.0071, 0.8727]}]}, {root: "lbr_iiwa_link_2", link: "lbr_iiwa_link_3", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.9671, -2.8798, -1.7453, 2.9671, 2.8798, 1.7453]}]},{root: "lbr_iiwa_link_3", link: "lbr_iiwa_link_4", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.0944, -2.0071, -0.8727, 2.0944, 2.0071, 0.8727]}]}, {root: "lbr_iiwa_link_4", link: "lbr_iiwa_link_5", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.9671, -2.8798, -1.7453, 2.9671, 2.8798, 1.7453]}]}, {root: "lbr_iiwa_link_5", link: "lbr_iiwa_link_6", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  -2.0944, -2.0071, -0.8727, 2.0944, 2.0071, 0.8727]}]}, {root: "lbr_iiwa_link_6", link: "lbr_iiwa_link_7", geometries: [{type: 7, data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0543, -2.9671, -1.7453, 3.0543, 2.9671, 1.7453]}]} ]}'

#set the collision objects
rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "world", link: "world", geometries: [{type: 3, data: [0.0, 0.0, 1.0, 0.2]}]}, {root: "world", link: "lbr_iiwa_link_3", geometries: [{type: 10, data: [0.0, 0.0, 0.0, 0.075]}]}, {root: "world", link: "lbr_iiwa_link_4", geometries: [{type: 10, data: [0.0, 0.0, 0.0, 0.12]}]}, {root: "world", link: "lbr_iiwa_link_5", geometries: [{type: 10, data: [0.0, 0.0, 0.0, 0.075]}]}, {root: "world", link: "lbr_iiwa_link_6", geometries: [{type: 10, data: [0.0, 0.0, 0.0, 0.1]}]}, {root: "world", link: "velvet_fingers_palm", geometries: [{type: 10, data: [0.04, 0.0, 0.025, 0.11]}]} ]}'

#set the joint setpoint objects
rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "lbr_iiwa_link_0", link: "lbr_iiwa_link_1", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_1", link: "lbr_iiwa_link_2", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_2", link: "lbr_iiwa_link_3", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_3", link: "lbr_iiwa_link_4", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_4", link: "lbr_iiwa_link_5", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_5", link: "lbr_iiwa_link_6", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]}, {root: "lbr_iiwa_link_6", link: "lbr_iiwa_link_7", geometries: [{type: 6, data: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}]} ]}'

#visualize the collision objects
rosservice call /apple/apple_hqp_vel_controller/visualize_task_objects '{ids: [7, 8, 9, 10, 11, 12]}'

#set the joint limit object tasks
rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 3, priority: 1, sign: "<=", t_obj_ids: [0], dynamics: {type: 1, data: [1.4835]}}, {type: 3, priority: 1, sign: "<=", t_obj_ids: [1], dynamics: {type: 1, data: [1.4835]}},{type: 3, priority: 1, sign: "<=", t_obj_ids: [2], dynamics: {type: 1, data: [1.7453]}},{type: 3, priority: 1, sign: "<=", t_obj_ids: [3], dynamics: {type: 1, data: [1.3090]}},{type: 3, priority: 1, sign: "<=", t_obj_ids: [4], dynamics: {type: 1, data: [2.2689]}},{type: 3, priority: 1, sign: "<=", t_obj_ids: [5], dynamics: {type: 1, data: [2.2689]}},{type: 3, priority: 1, sign: "<=", t_obj_ids: [6], dynamics: {type: 1, data: [2.2689]}} ]}'

#set the collision avoidance tasks
rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 9, priority: 1, sign: ">=", t_obj_ids: [8, 7], dynamics: {type: 1, data: [-0.6]}}, {type: 9, priority: 1, sign: ">=", t_obj_ids: [9, 7], dynamics: {type: 1, data: [-0.6]}}, {type: 9, priority: 1, sign: ">=", t_obj_ids: [10, 7], dynamics: {type: 1, data: [-0.6]}}, {type: 9, priority: 1, sign: ">=", t_obj_ids: [11, 7], dynamics: {type: 1, data: [-0.6]}}, {type: 9, priority: 1, sign: ">=", t_obj_ids: [12, 7], dynamics: {type: 1, data: [-0.6]}} ]}'

#set the joint setpoint tasks
rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 2, priority: 2, sign: "=", t_obj_ids: [13], dynamics: {type: 1, data: [-0.6]}}, {type: 2, priority: 2, sign: "=", t_obj_ids: [14], dynamics: {type: 1, data: [-0.6]}}, {type: 2, priority: 2, sign: "=", t_obj_ids: [15], dynamics: {type: 1, data: [-0.6]}},{type: 2, priority: 2, sign: "=", t_obj_ids: [16], dynamics: {type: 1, data: [-0.6]}}, {type: 2, priority: 2, sign: "=", t_obj_ids: [17], dynamics: {type: 1, data: [-0.6]}},{type: 2, priority: 2, sign: "=", t_obj_ids: [18], dynamics: {type: 1, data: [-0.6]}}, {type: 2, priority: 2, sign: "=", t_obj_ids: [19], dynamics: {type: 1, data: [-0.6]}} ]}'

rosservice call /apple/apple_hqp_vel_controller/activate_hqp_control true



