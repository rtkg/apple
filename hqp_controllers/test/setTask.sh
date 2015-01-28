#!/bin/bash

rosservice call /gazebo/set_physics_properties '{gravity: [0, 0, 0]}' 

rosservice call /apple/apple_hqp_vel_controller/set_task_object '{obj: {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 1, data: [0.1, 0.2, 0.3]}]  }}'
rosservice call /apple/apple_hqp_vel_controller/set_task_object '{obj: {root: "citi_truck_base", link: "citi_truck_base", geometries: [{type: 3, data: [0.0, 0.0, 1.0,0.2]}]  }}'
rosservice call /apple/apple_hqp_vel_controller/set_task '{task: {type: 1, priority: 1, sign: "=", t_obj_ids: [0, 1], dynamics: {type: 1, data: [0.0, 0.125, -16.5684, -2.8782]} } }'


