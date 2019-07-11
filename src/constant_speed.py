#!/usr/bin/env python2
# FILE			: constant_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 10 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import math

# Constant for first mission
_GATE_START_FORWARD_TIME_ = 6 
_GATE_START_FORWARD_DISTANCE_ = 3
_GATE_START_SURVEY_TIME_ = 6 
_GATE_START_SURVEY_DISTANCE_ = 3
_GATE_START_SURVEY_DIRECTION_ = 1.0 # positive is left and negative is right
_GATE_FORWARD_ONLY_TIME_ = 15
_GATE_FORWARD_ONLY_DISTANCE_ = 8
_GATE_START_DEPTH_ = -0.75

# Constant for mission path
_PATH_FIND_TIME_ = 5 # Left is constant but right + 2 second
_PATH_FORCE_X_ = 1
_PATH_FORCE_Y_ = 1.5
_PATH_FORCE_YAW_ = 0.3
_PATH_LAST_TIME_ = 8

# Constant for mission buoy
_BUOY_TIME_TO_BACK_ = 6
_BUOY_TIME_TO_SURVEY_ = 10
_BUOY_FORCE_SURVEY_ = 2
_BUOY_FORCE_FORWARD_ = 1.8

# Constant for connect mission
_STRATEGY_TIME_GATE_PATH_ = 25
_STRATEGY_FORCE_GATE_PATH_ = 2
_STRATEGY_ROTATION_GATE_BUOY_ = math.pi / 2
_STRATEGY_TIME_BUOY_ = 15
_STRATEGY_FORCE_BUOY_ = 1.5  
_STRATEGY_FORCE_BUOY_PATH_ = 15
_STRATEGY_TIME_BUOY_PATH_ = 2
_STRATEGY_ROTATION_BUOY_DROP_ = -1.0*math.pi / 4
