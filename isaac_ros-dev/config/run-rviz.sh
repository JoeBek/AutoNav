#!/bin/bash

NAV_PATH="$(dirname ${BASH_SOURCE[0]})/../src/sim/config/view_bot.rviz"

rviz2 -d $NAV_PATH
