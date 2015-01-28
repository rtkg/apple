#!/bin/bash

rosservice call /apple/apple_hqp_vel_controller/set_task_object '{obj: {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 4, data: [1, 2, 3, 0.2, 0.3, -0.1]}]  }}'



