#!/bin/bash

rosservice call /apple/apple_hqp_vel_controller/set_tasks '{tasks: [{type: 1, priority: 1, sign: "=", t_obj_ids: [0, 2], dynamics: {type: 1, data: [0.0, 0.125, -16.5684, -2.8782]}} ]}'







