#!/usr/bin/env python2
# FILE			: constant_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 10 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import math

# Constant for first mission
GATE_START_FORWARD_TIME_ = 9 
GATE_START_FORWARD_DISTANCE_ = 3
GATE_START_SURVEY_TIME_ = 15
GATE_START_SURVEY_DISTANCE_ = 4
GATE_START_SURVEY_DIRECTION_ = -1.0 # positive is left and negative is right
GATE_FORWARD_ONLY_TIME_ = 15
GATE_FORWARD_ONLY_DISTANCE_ = 8
GATE_START_DEPTH_ = -0.75
GATE_FORCE_Y = 180.0 # unit is / 100 kgf
GATE_FORCE_X = 120.0 # unit is / 100 kgf

# Constant for mission path
PATH_FIND_TIME_ = 5 # Left is constant but right + 2 second
PATH_FORCE_X_ = 1
PATH_FORCE_Y_ = 1.5
PATH_FORCE_YAW_ = 0.3
PATH_LAST_TIME_ = 4

# Constant for mission buoy
BUOY_TIME_TO_BACK_ = 3
BUOY_TIME_TO_SURVEY_ = 4
BUOY_FORCE_SURVEY_ = 2
BUOY_FORCE_FORWARD_ = 1.8
BUOY_FOUND_PICTURE_ = 1

# Constant for operator drop
DROP_HAVE_TO_ROTATION = False
DROP_RADIAN_TO_ROTATION = math.pi 
DROP_START_DEPTH = -1.0 # operator to run this file
DROP_TARGET_DEPTH = -2.0 # For doing individual mission drop or open
DROP_DEPTH_ACTION = -3.0 # depth when you want to drop or open
DROP_WANT_OPEN = False
DROP_FORCE_BACKWARD = -1.5

# Constant for connect mission
STRATEGY_TIME_GATE_PATH_ = 25
STRATEGY_FORCE_GATE_PATH_ = 2
STRATEGY_ROTATION_GATE_BUOY_ = math.pi / 4
STRATEGY_TIME_BUOY_ = 5
STRATEGY_FORCE_BUOY_ = 1.5  
STRATEGY_FORCE_BUOY_PATH_ = 15
STRATEGY_TIME_BUOY_PATH_ = 2
STRATEGY_ROTATION_BUOY_DROP_ = -1.0*math.pi / 4
STRATEGY_DEPTH_FIND_DROP = -0.5
