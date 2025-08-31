#!/bin/bash

ros2 run --prefix 'gdb --args' domabot_controller domabot_controller_node --ros-args -r __ns:=/domabot --log-level domabot.domabot_controller:=DEBUG --log-level domabot.domabot_controller.Modbus:=INFO --disable-rosout-logs --params-file ../config/default.yaml