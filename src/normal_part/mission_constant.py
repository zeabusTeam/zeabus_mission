#!/usr/bin/env python2
# FILE			: mission_constant.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 05 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import math

# Constant for strategy_speed
    # Part of process pass gate
STRATEGY_SPEED_FORCE_GATE_PATH = 1.5
STRATEGY_SPEED_TIME_GATE_PATH = 30
STRATEGY_SPEED_ROTATE_GATE_BUOY = -1.0*( math.pi / 2) # unit radian
    # Part of process init for buoy
STRATEGY_SPEED_FORCE_START_BUOY = 1.5
STRATEGY_SPEED_TIME_BUOY = 10
    # Part of process pass buoy
STRATEGY_SPEED_FORCE_BUOY_PATH = 1.5
STRATEGY_SPEED_TIME_BUOY_PATH = 15
STRATEGY_SPEED_ROTATE_BUOY_DROP = 1.0*( math.pi / 2) # unit radian

# Constant for mission gate first gate
GATE_START_FORWARD = 2
GATE_START_SURVEY = -1
GATE_LIMIT_TIME = 60
GATE_LIMIT_DISTANCE = 6
GATE_FORCE_X = 150 # unit (10^-2) kgf
GATE_FORCE_Y = 100 # unit (10^-2) kgf

# Constant for mission path 2 path
PATH_FORCE_X = 1.0
PATH_FORCE_Y = 1.5
PATH_PASS_TIME = 2
PATH_PASS_FORCE = 2.0
