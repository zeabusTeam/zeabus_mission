#!/usr/bin/env python2
# FILE			: strategy_straight.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE
#   ref01   : http://wiki.ros.org/rospy_tutorials/Tutorials/Logging
#   ref02   : http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

from __future__ import print_function

import rospy
import math

# For doing gate mission
#from gate_lib import Gate
#from zeabus_utility.srv import VisionGate, SendBool

# For doing gate mission by GAP
from gate import Gate

# For doing path mission
from path_speed import Path
from zeabus.vision.analysis_path import AnalysisPath

# For doing buoy mission
from buoy_speed import Buoy
from zeabus.vision.analysis_buoy import AnalysisBuoy

# Standard for connect with control
from zeabus.control.command_interfaces import CommandInterfaces

# Import math bound radian
from zeabus.math import general as zeabus_math

from zeabus_utility.srv import SendBoolResponse, SendBool

from mission_constant import *

class StrategySpeed:

    def __init__( self ):
        self.control = CommandInterfaces( "strategy" )

        self.control.publish_data( "Waiting service name /vision/gate" )
        rospy.wait_for_service('/vision/gate')

        self.rate = rospy.Rate( 10 )

        # Step setup mission Gate
        self.mission_gate = Gate()

        # Step setup mission Path 
        self.mission_path = Path( 0 , 0)
        self.vision_path = AnalysisPath()

        # Step setup mission Buoy
        self.mission_buoy = Buoy()
        self.vision_buoy = AnalysisBuoy()

        self.current_play = False

        # Step setup service of strategy_mission
        self.service_service = rospy.Service(
            '/mission/strategy' , SendBool() , self.callback_service )

        self.control.publish_data( "Waiting command to run mission")
        while( not rospy.is_shutdown() ):
            if( self.current_play ):
                self.main()
                self.current_play = False
                break
            self.rate.sleep()

    # Start part of all mission

    def main( self ):

        self.control.reset_state()
        self.control.publish_data( "STRATEGY I will start run mission now")

        self.mission_gate.start_mission()

        self.control.publish_data( "Finish mission gate I will move forward with serach path")

        # This step will use to movement forward
        self.control.update_target()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 

        self.control.deactivate( ['x' , 'y' ] )
        count = 0
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_SPEED_TIME_GATE_PATH ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                self.control.get_state()
                count += 1
            else:
                count = 0

            if( count == 2 ):
                self.control.publish_data("!!!!!!!!! STRATEGY FIND PATH !!!!!!!!!!!!!" )
                target_depth = -1.0
                while( not rospy.is_shutdown() ):
                    self.rate.sleep()
                    self.vision_path.call_data()
                    self.vision_path.echo_data()

                    relative_x = 0
                    relative_y = 0
                    ok_x = False
                    ok_y = False
                    
                    if( self.vision_path.num_point == 0 ):
                        relative_y = 0
                    elif( self.vision_path.x_point[0] > 30 ):
                        relative_y = -PATH_FORCE_Y
                    elif( self.vision_path.x_point[0] < -30 ):
                        relative_y = PATH_FORCE_Y
                    else:
                        ok_y = True

                    if( self.vision_path.num_point == 0):
                        relative_x = 0
                    elif( self.vision_path.y_point[0] > 20 ):
                        relative_x = PATH_FORCE_X
                    elif( self.vision_path.y_point[0] < -20 ):
                        relative_x = -PATH_FORCE_X
                    else:
                        ok_x = True

                    if( ok_x and ok_y ):
                        self.control.force_xy( 0 , 0 )
                        if( self.control.check_z( 0.15 ) ):
                            if( target_depth < -2.4 ):
                                self.control.publish_data( "Breaking and go to setup point")
                                break
                            else:
                                target_depth -= 0.5
                                self.control.publish_data( "I command depth to " 
                                    + str( target_depth ) )
                                self.control.absolute_z( target_depth )
                    else:
                        self.control.publish_data( "STRATEGY command " 
                            + repr(( relative_x , relative_y ) ) )
                        self.control.force_xy( relative_x , relative_y )

                self.control.force_xy( 0 , 0 )
                break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
            self.control.publish_data( "STRATEGY forward time is " + str( diff_time ) )
            self.control.force_xy( STRATEGY_SPEED_FORCE_GATE_PATH , 0 )

        self.control.activate( ['x' , 'y'] )

        # Part to move forward for path If you see path

        result = False

        if( count == 2 ):
            self.control.publish_data( "I start path by ever found path" )
            self.mission_path.setup_point()
            result = True

        if( result ):
            self.control.publish_data( "Congratulation we know you pass path")
        else:
            self.control.publish_data( "It bad you failure mission path" )
            self.control.relative_yaw( STRATEGY_SPEED_ROTATE_GATE_BUOY )

        self.control.publish_data( "I will waiting yaw before do next process")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        # Part of mission buoy

        self.control.publish_data( "STRATEGY start forward for init do mission buoy" )
        self.control.deactivate( ['x' , 'y'] )
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        count_found = 0
        pass_buoy = False
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_SPEED_TIME_BUOY ):
            self.rate.sleep()
            self.vision_buoy.call_data()
            self.vision_buoy.echo_data()
        
            if( self.vision_buoy.result[ 'found' ] ):
                count_found += 1
                if( count_found == 5 ):
                    self.control.publish_data( "STRATEGY I will call mission_buoy lock_target")
                    self.mission_buoy.lock_target()
                    pass_buoy = True
                else:
                    self.control.publish_data( "STRATEGY count found buoy is " 
                        + str( count_found ) )
                force_x = 0 
                force_y = 0
                if( self.vision_buoy.result[ 'center_x'] < -20 ):
                    force_y = 1.0
                elif( self.vision_buoy.result[ 'center_x'] > 20 ):
                    force_y = -1.0
                else:
                    force_x = 1.5
                self.control.publish_data( "Command force_xy is " + repr( (force_x , force_y) ) )
                self.control.force_xy( force_x , force_y )

            else:
                count_found = 0
                self.control.force_xy( STRATEGY_SPEED_FORCE_START_BUOY , 0 )

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                
        self.control.activate( ['x' , 'y'] )

        if( not pass_buoy ):
            self.control.publish_data( "STRATEGY will give process to mission_buoy start")
            self.mission_buoy.start_mission()
        else:
            self.control.publish_data( "Finish buoy next I will straight for search path")
    
        # Start path move forward and searching
        self.control.update_target()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 

        self.control.deactivate( ['x' , 'y' ] )
        count = 0
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_SPEED_TIME_BUOY_PATH ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                self.control.get_state()
                count += 1
            else:
                count = 0

            if( count == 2 ):
                self.control.publish_data("!!!!!!!!! STRATEGY FIND PATH !!!!!!!!!!!!!" )
                target_depth = -1.0
                while( not rospy.is_shutdown() ):
                    self.rate.sleep()
                    self.vision_path.call_data()
                    self.vision_path.echo_data()

                    relative_x = 0
                    relative_y = 0
                    ok_x = False
                    ok_y = False
                    
                    if( self.vision_path.num_point == 0 ):
                        rospy.logfatal("STRATGY path disappear noooooooo")
                        count = 0
                        relative_y = 0
                        break
                    elif( self.vision_path.x_point[0] > 30 ):
                        relative_y = -PATH_FORCE_Y
                    elif( self.vision_path.x_point[0] < -30 ):
                        relative_y = PATH_FORCE_Y
                    else:
                        ok_y = True

                    if( self.vision_path.num_point == 0):
                        relative_x = 0
                    elif( self.vision_path.y_point[0] > 20 ):
                        relative_x = PATH_FORCE_X
                    elif( self.vision_path.y_point[0] < -20 ):
                        relative_x = -PATH_FORCE_X
                    else:
                        ok_x = True

                    if( ok_x and ok_y ):
                        self.control.force_xy( 0 , 0 )
                        if( self.control.check_z( 0.15 ) ):
                            if( target_depth < -2.4 ):
                                self.control.publish_data( "Breaking and go to setup point")
                                break
                            else:
                                target_depth -= 0.5
                                self.control.publish_data( "I command depth to " 
                                    + str( target_depth ) )
                                self.control.absolute_z( target_depth )
                    else:
                        self.control.publish_data( "STRATEGY command " 
                            + repr(( relative_x , relative_y ) ) )
                        self.control.force_xy( relative_x , relative_y )

                self.control.force_xy( 0 , 0 )
                break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
            self.control.publish_data( "STRATEGY forward time is " + str( diff_time ) )
            self.control.force_xy( STRATEGY_SPEED_FORCE_BUOY_PATH , 0 )

        # End part to search

        self.control.activate( ['x' , 'y'] )
        
        result = False
        if( count == 2 ):
            self.control.publish_data( "I start path by ever found path" )
            self.mission_path.setup_point()
            result = True

        if( result ):
            self.control.publish_data( "Congratulation we know you pass path")
        else:
            self.control.publish_data( "It bad you failure mission path rotation to drop" )
            self.control.relative_yaw( STRATEGY_SPEED_ROTATE_BUOY_DROP )

        self.control.publish_data( "I will waiting yaw before do next process")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

    # End part of play all mission

        self.control.publish_data( "Finish all strategy mission" )
        self.control.deactivate( ["x", "y", "z", "roll", "pitch", "yaw"])

    def callback_service( self , request ):

        if( request.data == True ): # Want to play
            if( not self.current_play ):
                self.current_play = True
            else:
                rospy.logfatal( "Warning node alredy play you can't do that" )
        else:
            rospy.loginfo( "Service call to code node")
            rospy.signal_shutdown( "Service call to close or stop mission")

        return SendBoolResponse()

if __name__=="__main__":
    rospy.init_node('strategy_mission')

    mission = StrategySpeed()

