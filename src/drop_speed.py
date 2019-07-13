#!/usr/bin/env python2
# FILE			: drop_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 13 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
from zeabus_math import general as zeabus_math  
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_drop import AnalysisDrop

class Drop:

    def __init__( self ):

        self.vision = AnalysisDrop( "base_drop" )
        self.control = CommandInterfaces( "DROP")
