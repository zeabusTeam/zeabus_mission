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
GATE_START_DEPTH = -0.3            # This use to target depth of this mission
GATE_START_FORWARD_TIME = 7         # This is limit of time to go forward
GATE_START_FORWARD_DISTANCE = 3     # This is limit of distance to go forward
GATE_START_SURVEY_TIME = 3         # This is limit time for go servey 
GATE_START_SURVEY_DISTANCE = 6      # This is limit time for go survey 
GATE_START_SURVEY_DIRECTION = 1.0   # positive is left and negative is right
# ----> two below variable we use to sure we can 
GATE_FORWARD_ONLY_TIME = 25         # This is limit time for go direct. 
GATE_FORWARD_ONLY_DISTANCE = 8      # This is limit distane for go direct
GATE_FORCE_Y = TARGET_LEFT * 100    # unit is / 100 kgf
GATE_FORCE_X = SURVEY_FORWARD * 100 # unit is / 100 kgf
GATE_APPROVE_AGAIN = True           # This function help you. 
                                    # If you are mode last move will be call data again
# Constant for mission path
PATH_START_DEPTH = -1               # This will tell desire depth when want to start this mission
PATH_TARGET_DEPTH = -2.2            # This target of path will want to go before do mission
PATH_FIND_TIME = 5                  # Left is constant but right + 2 second
PATH_FORCE_YAW = 0.2                # This variable is force to use rotation
PATH_OK_DIFF_YAW = 0.16             # This use ok yaw to don't rotation again 
PATH_LAST_TIME = 8
PATH_END_DEPTH = -1.5               # This is absolute depth after do task path finish
PATH_MODE = True                    # If true mean you don't use mode tracking to each point

# Constant for mission buoy
BUOY_START_DEPTH = -2.4             # This will tell desire depth when want to start this mission
BUOY_FOUND_PICTURE = 1              # This use will deicision to change mode after found round
BUOY_TIME_LOCK_TARGET = 60          # This is limit you still in mode lock target
BUOY_AREA_ABORT = 8                 # In mode lock target you can out of loop by area
BUOY_LIMIT_TIME = 15                # This is limit time of dash mode only
# ----> Below constant variable is use after finish dash mode
BUOY_TARGET_DEPTH_FINISH = -1     # This is absolute depth when you finish dash mode
BUOY_FORCE_SURVEY = SURVEY_RIGHT               # force to survey after dash mode
BUOY_FORCE_FORWARD = SURVEY_FORWARD            # force to forward and backward after dahs mode
BUOY_TIME_TO_BACK = 3               # forward will plus 5 second
BUOY_TIME_TO_SURVEY = 6             # time to survey make increase opportunity to find path

# Constant for operator drop
# ----> Below 2 constan variable will help you to manage about flip mission or task 
DROP_HAVE_TO_ROTATION = False
DROP_RADIAN_TO_ROTATION = math.pi
# ----> Below 4 constant variable will connect about depth to manage or doing process
DROP_FIND_DEPTH = -0.5
DROP_START_DEPTH = -1.0             # Operator to do about search and start to find mission
DROP_TARGET_DEPTH = -1              # Operator to using doing sub mission ( drop or open)
DROP_ONLY_DEPTH = -2.5              # Depth for using guess drop garlic
DROP_ACTION_DEPTH = -3.4            # Depth for using open action by survey
DROP_STEP_DEPTH = -0.3              # Depth is use for relative depth to command
DROP_BACKWARD_TIME = 5              # This time to backward before drop
DROP_BACKWARD_FORCE=SURVEY_BACKWARD # This is force to backward before drop
# ----> Below constant in mission will may use for case you see all
DROP_WANT_OPEN = False              # This variable use to consider you want to try open or not
DROP_FORCE_OPEN = -1.5              # This force will use survey open
DROP_TIME_OPEN = 4                  # This is time to command same force
# ----> Drop force yaw
DROP_FORCE_YAW = 0.2         
# ----> Below 3 constant is use about estimate center to do mission
DROP_CENTER_X_DROP = -20             # POSITIVE FOR DROP LEFT
DROP_CENTER_X_OPEN = 40
DROP_CENTER_Y = 55 # target of center y when you want to drop and can use for estimate open

# Constant for operator exposed
EXPOSED_START_DEPTH = -0.75
EXPOSED_TARGET_DEPTH = -1.5
EXPOSED_LIMIT_ROUND = 6             # This is limit round to find object
EXPOSED_LIMIT_TIME = 10             # This is limit time to survey
EXPOSED_FORCE_YAW = 0.2             # This is force to use rotation
# ====> specific about check coffin
EXPOSED_FORCE_TO_FIND = SURVEY_LEFT
EXPOSED_FORCE_TO_BACK = SURVEY_RIGHT
# ====> Two below constant have relative
# Please warning about wall please design direction to non wall 
EXPOSED_CENTER_X_DIRECTION = -1.0    # positive -1 in case you survey left and 1 in case right
EXPOSED_CENTER_X_NEW_VALUE = 70
EXPOSED_LIMIT_TIME_TO_FIND = 15 

# Constant for operator stake
STAKE_START_DEPTH = -2.2
STAKE_Z_DOWN = -3.2
STAKE_Z_FORCE_0 = -2.8
STAKE_Z_UP = -2.4
STAKE_AREA_ROTATION = 15
STAKE_AREA_ROTATION_OVER = 30
STAKE_OVAL_DIRECTION = 'left'
if( STAKE_OVAL_DIRECTION == 'right' ):
    STAKE_OVAL_CENTER_X = 50
else:
    STAKE_OVAL_CENTER_X = -50
STAKE_OVAL_AREA = 6
STAKE_VAMPIRE_AREA = 18
STAKE_LIMIT_TIME = 10
STAKE_HEART_AREA = 3
STAKE_TARGET_POINT = ( -50.1 , 11.2 )
STAKE_TARGET_X = ( STAKE_TARGET_POINT[0] - 10 , STAKE_TARGET_POINT[0] + 10 )
STAKE_TARGET_Y = ( STAKE_TARGET_POINT[1] - 10 , STAKE_TARGET_POINT[1] + 10 ) 
STAKE_BACKWARD_TIME = 4
STAKE_HEART_CENTER_Y = -10

# Constant for operator drop
DROP_HAVE_TO_ROTATION = False
DROP_RADIAN_TO_ROTATION = math.pi 
DROP_START_DEPTH = -1.5
DROP_DEPTH_ACTION = -3 # depth when you want to drop or open

# Constant for connect mission
# ====> Mission Path 
STRATEGY_NO_PATH = False
# ====> Mission Gate
STRATEGY_TIME_SURVEY_PATH = 3               # Time to survey before pass gate
STRATEGY_FORCE_SURVEY_PATH = SUPER_RIGHT    # Force to survey before pass gate
STRATEGY_TIME_GATE_PATH = 30                # Time to forward and doing pass gate
STRATEGY_FORCE_GATE_PATH = SUPER_FORWARD    # Force to forward and doing pass gate
STRATEGY_ROTATION_GATE_BUOY = math.pi / 8   # Use rotation when don't found path
STRATEGY_FIX_YAW_GATE = False                # If this true don't care above variable
# ====> Mission Buoy
STRATEGY_DEPTH_BOUY = BUOY_START_DEPTH      # This depth will use in buoy mission
STRATEGY_TIME_BUOY = 25                      # This is time to forward search direct after path
STRATEGY_FORCE_BUOY = SURVEY_FORWARD        # This is force forward search direct after path
STRATEGY_TIME_BUOY_SURVEY = 2             # This is time to survey after path to buoy
STRATEGY_FORCE_BUOY_SURVEY = SURVEY_LEFT    # This is force survey after path to buoy
STRATEGY_TIME_BUOY_PATH = 10                # This is time to use forward find buoy only direct
STRATEGY_FORCE_BUOY_PATH = SURVEY_FORWARD   # This is force use to direct after buoy to search path
# ====> Mission DROP
STRATEGY_ROTATION_BUOY_DROP = -math.pi/4  # This rotation buoy to drop in case don't find path
STRATEGY_DEPTH_FIND_DROP = DROP_START_DEPTH  # This use to depth for find depth
STRATEGY_FREE_TIME_DROP = 5
STRATEGY_FREE_FORCE_DROP = SURVEY_FORWARD
STRATEGY_FORCE_SURVEY_DROP = SURVEY_LEFT
STRATEGY_TIME_SURVEY_DROP = 2
STRATEGY_FORCE_DROP = SURVEY_FORWARD
STRATEGY_TIME_DROP = 20
STRATEGY_DISTANCE_DROP = 6
# ====> Choice to do last mission have 3 choind
STRATEGY_CHOICE_PROCESS = 0
STRATEGY_ROTATION_EXPOSED = math.pi/8.0     # This use to rotation from drop to exposed
STRATEGY_ROTATION_STAKE = math.pi/8.0       # This will use collect to command absolute yaw
#========> 0 is No use dvl : 1 is use DVL : 2 is use hydophone
# ====> Mission Exposed
STRATEGY_EXPOSED_FIND = EXPOSED_START_DEPTH
STRATEGY_TIME_SURVEY = 2
STRATEGY_DISTANCE_SURVEY = 3
STRATEGY_FORCE_SURVEY = SURVEY_LEFT
STRATEGY_TIME_FORWARD = 30
STRATEGY_FORCE_FORWARD = SURVEY_FORWARD
STRATEGY_DISTANCE_FORWARD = 10
# ====> Mission Stake
STRATEGY_STAKE_DEPTH = STAKE_START_DEPTH
STRATEGY_STAKE_TIME_FORWARD = 25
STRATEGY_STAKE_FORCE_FORWARD = SURVEY_BACKWARD
STRATEGY_STAKE_DISTANCE_FORWARD = 10
STRATEGY_STAKE_TIME_SURVEY = 35
STRATEGY_STAKE_FORCE_SURVEY = SURVEY_LEFT
STRATEGY_STAKE_DISTANCE_SURVEY = 10
STRATEGY_STAKE_DEPTH_FIND = STAKE_START_DEPTH
STRATEGY_FORCE_STAKE = 1.0
STRATEGY_STAKE_AREA_FOCUS = STAKE_AREA_ROTATION - 3  # range 0 - 100

# For special characteristice ROBOSUB for doing mission buoy
ROBOSUB_TIME_SURVEY_TO_PATH = 8
ROBOSUB_FORCE_SURVEY_TO_PATH = SURVEY_LEFT
ROBOSUB_TIME_FORWARD_TO_PATH = 30
ROBOSUB_FORCE_FORWARD_TO_PATH = SURVEY_FORWARD

ROBOSUB_TIME_SURVEY_TRIANGLE_BUOY = 5
ROBOSUB_FORCE_SURVEY_TRIANGLE_BUOY = TARGET_LEFT
ROBOSUB_TIME_FORWARD_TRIANGLE_BUOY = 6
ROBOSUB_FORCE_FORWARD_TRIANGLE_BUOY = TARGET_FORWARD
# Auto backward
ROBOSUB_TIME_FIRST_SURVEY_SINGLE_BUOY = 10
ROBOSUB_FORCE_FIRST_SURVEY_SINGLE_BUOY = TARGET_RIGHT
ROBOSUB_ROTATION_SINGLE_BUOY = -math.pi / 2.0
ROBOSUB_TIME_SECOND_SURVEY_SINGLE_BUOY = 10
ROBOSUB_FORCE_SECOND_SURVEY_SINGLE_BUOY = TARGET_RIGHT
