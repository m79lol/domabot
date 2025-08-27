#!/bin/bash

ros2 run --prefix 'gdb --args' domabot_cli domabot_cli_node --ros-args -r __ns:=/domabot --log-level domabot.domabot_cli:=DEBUG --disable-rosout-logs