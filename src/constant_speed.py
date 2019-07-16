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
GATE_START_SURVEY_TIME_ = 20
GATE_START_SURVEY_DISTANCE_ = 4
GATE_START_SURVEY_DIRECTION_ = 1.0 # positive is left and negative is right
GATE_FORWARD_ONLY_TIME_ = 15
GATE_FORWARD_ONLY_DISTANCE_ = 8
GATE_START_DEPTH_ = -0.75
GATE_FORCE_Y = 180.0 # unit is / 100 kgf
GATE_FORCE_X = 120.0 # unit is / 100 kgf
GATE_APPROVE_AGAIN_ = True

# Constant for mission path
PATH_FIND_TIME_ = 5 # Left is constant but right + 2 second
PATH_FORCE_X_ = 1
PATH_FORCE_Y_ = 1.5
PATH_FORCE_YAW_ = 0.3
PATH_LAST_TIME_ = 4

# Constant for mission buoy
BUOY_TIME_TO_BACK_ = 3 # forward will plus 5 second
BUOY_TIME_TO_SURVEY_ = 5
BUOY_FORCE_SURVEY_ = 2
BUOY_FORCE_FORWARD_ = 1.3
BUOY_FOUND_PICTURE_ = 1

# Constant for operator drop
DROP_HAVE_TO_ROTATION_ = False
DROP_RADIAN_TO_ROTATION_ = math.pi 
DROP_START_DEPTH_ = -1.5 # operator to run this file
DROP_TARGET_DEPTH_ = -2.0 # For doing individual mission drop or open
DROP_ACTION_DEPTH_ = -3.0 # depth when you want to drop or open
DROP_ONLY_DEPTH_ = -2.5
DROP_STEP_DEPTH_ = -0.3
DROP_WANT_OPEN_ = False
DROP_FORCE_OPEN_ = -1.5
DROP_TIME_OPEN_ = 5
DROP_FORCE_BACKWARD_ = -1.5
DROP_OFFSET_DROP_ = -20
DROP_ONLY_ = 55 # target of center y when you want to drop

# Constant for operator stake
STAKE_Z_FORCE_0 = -2
STAKE_Z_DOWN = -2.3
STAKE_Z_UP = -1.8
STAKE_AREA_ROTATION = 20
STAKE_AREA_ROTATION_OVER = 40
STAKE_OVAL_DIRECTION = 'right'
if( STAKE_OVAL_DIRECTION == 'right' ):
    STAKE_OVAL_CENTER_X = 50
else:
    STAKE_OVAL_CENTER_X = -50
STAKE_OVAL_AREA = 6
STAKE_HEART_AREA = 25
STAKE_TARGET_POINT = ( -30 , 20 ) 

# Constant for connect mission
# ====> Mission Gate
STRATEGY_TIME_GATE_PATH_ = 25
STRATEGY_FORCE_GATE_PATH_ = 2
STRATEGY_ROTATION_GATE_BUOY_ = math.pi / 4
# ====> Mission Buoy
STRATEGY_DEPTH_BOUY_ = -2
STRATEGY_TIME_BUOY_ = 5
STRATEGY_FORCE_BUOY_ = 1.5  
STRATEGY_FORCE_BUOY_PATH_ = 1.2
STRATEGY_TIME_BUOY_PATH_ = 3
# ====> Mission DROP
STRATEGY_ROTATION_BUOY_DROP_ = -1.0*math.pi / 4
STRATEGY_DEPTH_FIND_DROP_ = -1
STRATEGY_FREE_TIME_DROP_ = 5
STRATEGY_FORCE_DROP_ = 1.5
STRATEGY_TIME_DROP_ = 20
STRATEGY_DISTANCE_DROP_ = 6
# ====> Mission Stake
STRATEGY_STAKE_DEPTH_FIND = -3
STRATEGY_FORCE_STAKE = 1.0
STRATEGY_STAKE_AREA_FOCUS = 10  # range 0 - 100
