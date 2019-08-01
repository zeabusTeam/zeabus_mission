#!/usr/bin/env python2
# FILE			: temp_import.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, August 01 (UTC+0)
# MAINTAINER	: K.Supasan

# README
# This file use only for testing import and checking syntax

# REFERENCE

from strategy_speed import StrategySpeed
from strategy_robosub import StrategyRobosub
import rospy

if __name__=="__main__":
    rospy.init_node('strategy_mission')

    mission = StrategySpeed()
