#!/bin/bash

rosservice call /apple/apple_hqp_vel_controller/set_task_objects '{objs: [{root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 1, data: [0.1, 0.2, 0.3]}]}, {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 2, data: [1, 2, 3, 4, 5, 6]}]}, {root: "citi_truck_base", link: "citi_truck_base", geometries: [{type: 3, data: [0.0, 0.0, -1.0, 0.1]}]}, {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 4, data: [1, 2, 3, 0.2, 0.3, -0.1]}]}, {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 5, data: [1, 2, 3, 4, 5, 6, 0.2]}]} ]}'





