#!/bin/bash

rosservice call /apple/apple_hqp_vel_controller/set_task_object '{obj: {root: "citi_truck_base", link: "lbr_iiwa_link_7", geometries: [{type: 2, data: [1, 2, 3, 4, 5, 6]}]  }}'



