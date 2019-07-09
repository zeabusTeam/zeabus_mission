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

        # Step setup mission gate
#        try:
#            gate_srv = rospy.ServiceProxy('/vision/gate', VisionGate)
#        except rospy.ServiceException, e:
#            print("Service call failed: %s" % e)
#        self.mission_gate = Gate(gate_srv)
        self.mission_gate = Gate()

        # Step setup mission Path 
        self.mission_path = Path( 0 , 0)
        self.vision_path = AnalysisPath()

        # Step setup mission buoy
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
        self.control.publish_data( "Now I will run code doing mission gate")

        self.mission_gate.start_mission()

        self.control.publish_data( "Finish to search gate I will move forward with serach path")

        # This step will use to movement forward
        self.control.update_target()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 

        self.control.deactivate( ['x' , 'y' ] )

        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_SPEED_TIME_GATH_PATH ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                self.control.get_state()
                count += 1
            else:
                count = 0

            if( count == 2 ):
                self.control.publish_data("!!!!!!!!1 STRATEGY FIND PATH !!!!!!!!!!!!!" )
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
            self.control.force_xy( STRATEGY_SPEED_FORCE_GATH_PATH , 0 )

        self.control.activate( ['x' , 'y'] )

        # Part to move forward for path

        result = False

        if( count == 2 ):
            self.control.publish_data( "I start path by ever found path" )
            self.mission_path.setup_point()
            result = True
        else:
            self.control.publish_data( "I start path by never found path" )

        if( result ):
            self.control.publish_data( "Congratulation we know you pass path")
        else:
            self.control.publish_data( "It bad you failure mission path" )
            self.control.relative_yaw( zeabus_math.bound_radian( (math.pi / 2) ) )

        self.control.relative_xy( 0 , 0 )

        self.control.publish_data( "Waiting yaw before send process to buoy_straight")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.mission_buoy.start_mission()

        self.control.publish_data( "Finish play buoy next I will play path move up")
        self.control.absolute_z( -1 )
        while( not self.control.check_z(0.15)):
            self.rate.sleep()
    
        self.control.publish_data( "Waiting xy")
        while( not self.control.check_xy(0.15,0.15)):
            self.rate.sleep()

        self.control.publish_data( "Waiting yaw")
        while( not self.control.check_yaw(0.15)):
            self.rate.sleep()

        self.control.publish_data( "Move forward and searching 2.5 meter" )
        self.control.relative_xy( 2.5 , 0 )

        # Start path move forward and searching
        count = 0
        while( not self.control.check_xy( 0.1 , 0.1 ) ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                count += 1
            else:
                count = 0

            if( count == 2 ):
                self.control.publish_data( "I found path 2 round at here reset now")
                self.control.reset_state()
                self.control.publish_data( "Sleep 2 second wait to reset state" )
                rospy.sleep( 2 )
                break
        # End part to search

        if( count == 2 ):
            self.control.publish_data( "I start path by ever found path" )
        else:
            self.control.publish_data( "I start path by never found path" )

        result = self.mission_path.find_path()

        if( result ):
            self.control.publish_data( "Congratulation we know you pass path")
        else:
            self.control.publish_data( "It bad you failure mission path" )
            self.control.relative_yaw( zeabus_math.bound_radian( -math.pi / 2) )

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

