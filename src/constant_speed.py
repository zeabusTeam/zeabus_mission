#!/usr/bin/env python2
# FILE			: constant_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 10 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import math

# Constant for manage about force
# TARGET when you want to manage center point
TARGET_FORWARD = 0.5
TARGET_BACKWARD = -0.5
TARGET_LEFT = 0.8
TARGET_RIGHT = -0.8
# SURVEY when you want to find object or target
SURVEY_FORWARD = 1.2
SURVEY_BACKWARD = -1.2
SURVEY_LEFT = 1.5
SURVEY_RIGHT = -1.5
# SUPER when you want to move don't care everything
SUPER_FORWARD = 2.0
SUPER_BACKWORD = -2.0
SUPER_LEFT = 2.3
SUPER_RIGHT = -2.3

# Constant for first mission is mission gate
GATE_START_DEPTH = -0.75            # This use to target depth of this mission
GATE_START_FORWARD_TIME = 9         # This is limit of time to go forward
GATE_START_FORWARD_DISTANCE = 3     # This is limit of distance to go forward
GATE_START_SURVEY_TIME = 20         # This is limit time for go servey 
GATE_START_SURVEY_DISTANCE = 6      # This is limit time for go survey 
GATE_START_SURVEY_DIRECTION = 1.0   # positive is left and negative is right
# ----> two below variable we use to sure we can 
GATE_FORWARD_ONLY_TIME = 30         # This is limit time for go direct. 
GATE_FORWARD_ONLY_DISTANCE = 8      # This is limit distane for go direct
GATE_FORCE_Y = TARGET_LEFT * 100    # unit is / 100 kgf
GATE_FORCE_X = SURVEY_FORWARD * 100 # unit is / 100 kgf
GATE_APPROVE_AGAIN = True           # This function help you. 
                                    # If you are mode last move will be call data again

# Constant for mission path
PATH_START_DEPTH = -1               # This will tell desire depth when want to start this mission
PATH_TARGET_DEPTH = -2.4            # This target of path will want to go before do mission
PATH_FIND_TIME = 5                  # Left is constant but right + 2 second
PATH_FORCE_YAW = 0.3                # This variable is force to use rotation
PATH_LAST_TIME = 8
PATH_END_DEPTH = -1.5               # This is absolute depth after do task path finish

# Constant for mission buoy
BUOY_START_DEPTH = -2.1             # This will tell desire depth when want to start this mission
BUOY_FOUND_PICTURE = 1              # This use will deicision to change mode after found round
BUOY_TIME_LOCK_TARGET = 60          # This is limit you still in mode lock target
BUOY_AREA_ABORT = 8                 # In mode lock target you can out of loop by area
BUOY_LIMIT_TIME = 15                # This is limit time of dash mode only
# ----> Below constant variable is use after finish dash mode
BUOY_TARGET_DEPTH_FINISH = -0.5     # This is absolute depth when you finish dash mode
BUOY_FORCE_SURVEY = 2               # force to survey after dash mode
BUOY_FORCE_FORWARD = 1.3            # force to forward and backward after dahs mode
BUOY_TIME_TO_BACK = 3               # forward will plus 5 second
BUOY_TIME_TO_SURVEY = 5             # time to survey make increase opportunity to find path

# Constant for operator drop
# ----> Below 2 constan variable will help you to manage about flip mission or task 
DROP_HAVE_TO_ROTATION = False
DROP_RADIAN_TO_ROTATION = math.pi
# ----> Below 4 constant variable will connect about depth to manage or doing process
DROP_FIND_DEPTH = -0.5 
DROP_START_DEPTH = -1.0             # Operator to do about search and start to find mission
DROP_TARGET_DEPTH = -1.5            # Operator to using doing sub mission ( drop or open)
DROP_ONLY_DEPTH = -2.5             # Depth for using guess drop garlic
DROP_ACTION_DEPTH = -2.8            # Depth for using open action by survey
DROP_STEP_DEPTH = -0.3             # Depth is use for relative depth to command
# ----> Below constant in mission will may use for case you see all
DROP_WANT_OPEN = False             # This variable use to consider you want to try open or not
DROP_FORCE_OPEN = -1.5             # This force will use survey open
DROP_TIME_OPEN = 8                 # This is time to command same force
DROP_FORCE_BACKWARD_ = -1.5
# ----> Drop force yaw
DROP_FORCE_YAW = 0.2         
# ----> Below 3 constant is use about estimate center to do mission
DROP_CENTER_X_DROP = 20
DROP_CENTER_X_OPEN = -20
DROP_CENTER_Y = 55 # target of center y when you want to drop and can use for estimate open

# Constant for operator stake
#
STAKE_Z_DOWN = -2.3
STAKE_Z_FORCE_0 = -2
STAKE_Z_UP = -1.7
STAKE_AREA_ROTATION = 15
STAKE_AREA_ROTATION_OVER = 30
STAKE_OVAL_DIRECTION = 'right'
if( STAKE_OVAL_DIRECTION == 'right' ):
    STAKE_OVAL_CENTER_X = 50
else:
    STAKE_OVAL_CENTER_X = -50
STAKE_OVAL_AREA = 6
STAKE_HEART_AREA = 3
STAKE_TARGET_POINT = ( -50.1 , 11.2 ) 
STAKE_BACKWARD_TIME = 4
STAKE_HEART_CENTER_Y = -10

# Constant for connect mission
# ====> Mission Gate
STRATEGY_TIME_SURVEY_PATH = 2               # Time to survey before pass gate
STRATEGY_FORCE_SURVEY_PATH = SUPER_RIGHT    # Force to survey before pass gate
STRATEGY_TIME_GATE_PATH = 20                # Time to forward and doing pass gate
STRATEGY_FORCE_GATE_PATH = SUPER_FORWARD    # Force to forward and doing pass gate
STRATEGY_ROTATION_GATE_BUOY = math.pi / 4   # Use rotation when don't found path
# ====> Mission Buoy
STRATEGY_DEPTH_BOUY = BUOY_START_DEPTH      # This depth will use in buoy mission
STRATEGY_TIME_BUOY = 10                     # This is time to forward search direct after path
STRATEGY_FORCE_BUOY = SURVEY_FORWARD        # This is force use to forward search direct after path
STRATEGY_TIME_BUOY_PATH = 10                # This is time to use forward find buoy only direct 
STRATEGY_FORCE_BUOY_PATH = SURVEY_FORWARD   # This is force use to direct after buoy to search path
# ====> Mission DROP
STRATEGY_ROTATION_BUOY_DROP = math.pi / -4 # This use rotation buoy to drop in case don't find path
STRATEGY_DEPTH_FIND_DROP = DROP_FIND_DEPTH # This use to depth for find depth
STRATEGY_FREE_TIME_DROP = 6
STRATEGY_FORCE_DROP = SURVEY_FORWARD
STRATEGY_TIME_DROP = 20
STRATEGY_DISTANCE_DROP = 6
# ====> Mission Stake
STRATEGY_STAKE_DEPTH_FIND = -3
STRATEGY_FORCE_STAKE = 1.0
STRATEGY_STAKE_AREA_FOCUS = 10  # range 0 - 100
