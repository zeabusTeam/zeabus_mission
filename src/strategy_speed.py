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
from gate_speed import Gate

# For doing path mission
from path_speed import Path
from zeabus.vision.analysis_path import AnalysisPath

# For doing buoy mission
from buoy_speed import Buoy
from zeabus.vision.analysis_buoy import AnalysisBuoy

# For doung drop mission
from drop_only import Drop
from zeabus.vision.analysis_drop import AnalysisDrop

# For doing exposed by analysis coffin mission
from exposed_speed import Exposed
from zeabus.vision.analysis_exposed import AnalysisCoffin

# For doing stake by analysis stake mission
from stake_not_open import Stake
from zeabus.vision.analysis_stake import AnalysisStake

# Standard for connect with control
from zeabus.control.command_interfaces import CommandInterfaces

# Import math bound radian
from zeabus.math import general as zeabus_math

from zeabus_utility.srv import SendBoolResponse, SendBool

# Import constant
from constant_speed import *
from zeabus.vision.analysis_constant import *

class StrategySpeed:

    def __init__( self ):
        self.control = CommandInterfaces( "strategy" )

        self.control.publish_data( "Waiting service name /vision/gate" )
        rospy.wait_for_service('/vision/gate')

        self.rate = rospy.Rate( 10 )

        # Step setup mission Gate
        self.mission_gate = Gate()

        # Step setup mission Path 
        self.mission_path = Path()
        self.vision_path = AnalysisPath()

        # Step setup mission Buoy
        self.mission_buoy = Buoy()
        self.vision_buoy = AnalysisBuoy()

        # Step setup mission Drop
        self.mission_drop = Drop()
        self.vision_drop = AnalysisDrop()

        # Step setup mission Exposed analysis coffin
        self.mission_exposed = Exposed()
        self.vision_coffin = AnalysisCoffin( "base_coffin" )

        self.mission_stake = Stake()
        self.vision_stake = AnalysisStake( "base_stake" )

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

        self.control.publish_data( "STRATEGY Finish gate next forward and found path")

        # This step will use to movement forward
        self.control.update_target()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 

        self.control.deactivate( ['x' , 'y' ] )

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_SURVEY_PATH ):
            self.rate.sleep()
            self.control.force_xy( 0 , STRATEGY_FORCE_SURVEY_PATH )
            self.control.publish_data( "STRATEGY survey before path " + str(diff_time)  ) 
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        
        count = 0
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()

        if STRATEGY_NO_PATH :
            self.control.publish_data( "STRATEGY no play path")

        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_GATE_PATH ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                count += 1
                self.control.force_xy( 0 , 0 )
            else:
                count = 0
                self.control.force_xy( STRATEGY_FORCE_GATE_PATH , 0 )

            diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
            self.control.publish_data( "STRATEGY forward time is " + str( diff_time ) )

            if( count == 3 ):

                if STRATEGY_NO_PATH :
                    self.control.publish_data( "!!!!! STRATEGY_NO_PATH FIND PATH !!!!!!!!")
                    self.control.force_xy( 0 , 0 )
                    break

                self.control.publish_data("!!!!!!!!! STRATEGY FIND PATH !!!!!!!!!!!!!" )
                target_depth = -1.0
                count_unfound = 0
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
                        relative_y = 0
                        count = 0
                        relative_y = 0
                        count_unfound += 1
                        if count_unfound == 3:
                            break
                        continue
                    elif( self.vision_path.x_point[0] > 30 ):
                        relative_y = SURVEY_RIGHT
                    elif( self.vision_path.x_point[0] < -30 ):
                        relative_y = SURVEY_LEFT
                    else:
                        ok_y = True
                    count_unfound = 0
                    if( self.vision_path.num_point == 0):
                        relative_x = 0
                    elif( self.vision_path.y_point[0] > 20 ):
                        relative_x = SURVEY_FORWARD
                    elif( self.vision_path.y_point[0] < -20 ):
                        relative_x = SURVEY_BACKWARD
                    else:
                        ok_x = True

                    if( ok_x and ok_y ):
                        self.control.force_xy( 0 , 0 )
                        if( self.control.check_z( 0.15 ) ):
                            if( target_depth < PATH_TARGET_DEPTH ):
                                self.control.publish_data( "STRATEGY Breaking and setup point")
                                break
                            else:
                                target_depth -= 0.5
                                self.control.publish_data( "STRATEGY I command depth to " 
                                    + str( target_depth ) )
                                self.control.absolute_z( target_depth )
                    else:
                        self.control.publish_data( "STRATEGY command " 
                            + repr(( relative_x , relative_y ) ) )
                        self.control.force_xy( relative_x , relative_y )

                self.control.force_xy( 0 , 0 )
                break

        self.control.relative_xy( 0 , 0 )
        self.control.activate( ['x' , 'y'] )

        # Part to move forward for path If you see path

        if( count == 3 ) and not STRATEGY_NO_PATH:
            self.control.publish_data( "STRATEGY start mission path on setup_point" )
            self.mission_path.setup_point()
        elif STRATEGY_NO_PATH : 
            self.control.publish_data( "STRATEGY don't play path")
            self.control.relative_yaw( STRATEGY_ROTATION_GATE_BUOY )
            self.control.sleep()
        else:
            self.control.publish_data( "STRATEGY don't found picture I will rotation" )
            self.control.relative_yaw( STRATEGY_ROTATION_GATE_BUOY )
            self.control.sleep()

        self.control.publish_data( "STRATEGY waiting yaw before start buoy")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        # Part of mission buoy
        self.control.publish_data( "STRATEGY Command depth " + str( STRATEGY_DEPTH_BOUY ) )
        self.control.absolute_z( STRATEGY_DEPTH_BOUY )
        self.control.sleep()
        while( ( not self.control.check_z( 0.12 ) ) ):
            self.rate.sleep()

        self.control.publish_data( "STRATEGY start survey for init do mission buoy" )
        self.control.deactivate( ['x' , 'y'] )
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_BUOY_SURVEY ):
            self.control.force_xy( 0 , STRATEGY_FORCE_BUOY_SURVEY )
            self.rate.sleep()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "STRATEGY Survey is diff time " + str( diff_time ) )

        self.control.force_xy( 0 , 0 )
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        count_found = 0
        pass_buoy = False
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_BUOY ):
            self.rate.sleep()
            self.vision_buoy.call_data()
            self.vision_buoy.echo_data()
        
            if( self.vision_buoy.result[ 'found' ] ):
                count_found += 1
                if( count_found == 3 ):
                    self.control.publish_data( "STRATEGY I will call mission_buoy lock_target")
                    self.mission_buoy.lock_target()
                    pass_buoy = True
                else:
                    self.control.publish_data( "STRATEGY count found buoy is " 
                        + str( count_found ) )
                force_x = 0 
                force_y = 0
                if( self.vision_buoy.result[ 'center_x'] < -30 ):
                    force_y = SURVEY_LEFT
                elif( self.vision_buoy.result[ 'center_x'] > 30 ):
                    force_y = SURVEY_RIGHT
                else:
                    force_x = TARGET_FORWARD 
                self.control.publish_data( "Command force_xy is " + repr((force_x , force_y)) )
                self.control.force_xy( force_x , force_y )

            else:
                self.control.publish_data( "STRATEGY I not found bouy")
                count_found = 0
                self.control.force_xy( STRATEGY_FORCE_BUOY , 0 )

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                
        self.control.activate( ['x' , 'y'] )

        if( not pass_buoy ):
            self.control.publish_data( "STRATEGY will give process to find_target")
            self.mission_buoy.find_target()
        else:
            self.control.publish_data( "STRATEGY Finish buoy I will go next for path")
    
        # Start path move forward and searching

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
        if STRATEGY_NO_PATH :
            self.control.publish_data( "STRATEGY Don't play path")
        else:
            self.control.publish_data( "STRATEGY command depth -1 meter")
            self.control.absolute_z( -1 )
            self.control.deactivate( ['x' , 'y' ] )
        count = 0
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_BUOY_PATH ):
            self.rate.sleep()
            self.vision_path.call_data()
            self.vision_path.echo_data()
            if( self.vision_path.num_point != 0 ):
                count += 1
                self.control.force_xy( 0 , 0 )
            else:
                self.control.force_xy( STRATEGY_FORCE_BUOY_PATH , 0 )
                count = 0

            diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
            self.control.publish_data( "STRATEGY forward time is " + str( diff_time ) )

            if( count == 4 ):

                if STRATEGY_NO_PATH :
                    self.control.publish_data("!!!!!!! STRATEGY_NO_PATH Find path !!!!!!!!")
                    self.control.force_xy( 0 , 0 )
                    break

                self.control.publish_data("!!!!!!!!! STRATEGY FIND PATH !!!!!!!!!!!!!" )
                target_depth = -1.0
                count_unfound = 0
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
                        count_unfound += 1
                        if count_unfound == 3:
                            break
                        continue
                    elif( self.vision_path.x_point[0] > 30 ):
                        relative_y = TARGET_RIGHT
                    elif( self.vision_path.x_point[0] < -30 ):
                        relative_y = TARGET_LEFT
                    else:
                        ok_y = True
                    count_unfound = 0 
                    if( self.vision_path.num_point == 0):
                        relative_x = 0
                    elif( self.vision_path.y_point[0] < -60 ):
                        relative_x = TARGET_BACKWARD * 2.0
                    elif( self.vision_path.y_point[0] > 20 ):
                        relative_x = TARGET_FORWARD
                    elif( self.vision_path.y_point[0] < -20 ):
                        relative_x = TARGET_BACKWARD
                    else:
                        ok_x = True

                    if( ok_x and ok_y ):
                        self.control.force_xy( 0 , 0 )
                        if( self.control.check_z( 0.15 ) ):
                            if( target_depth < PATH_TARGET_DEPTH ):
                                self.control.publish_data( "Breaking and go to setup point")
                                break
                            else:
                                target_depth -= 0.5
                                self.control.publish_data( "STRATGY I command depth to " 
                                    + str( target_depth ) )
                                self.control.absolute_z( target_depth )
                    else:
                        self.control.publish_data( "STRATEGY command " 
                            + repr(( relative_x , relative_y ) ) )
                        self.control.force_xy( relative_x , relative_y )

                self.control.force_xy( 0 , 0 )
                break

        # End part to search parh

        self.control.activate( ['x' , 'y'] )
        self.control.relative_xy( 0 , 0 )
        self.control.sleep()
        
        if( count == 4 ) and not STRATEGY_NO_PATH:
            self.control.publish_data( "I start path by ever found path" )
            self.mission_path.setup_point()
        elif STRATEGY_NO_PATH :
            self.control.publish_data( "STRATEGY No play path")
            self.control.relative_yaw( STRATEGY_ROTATION_BUOY_DROP )
            self.control.sleep()
        else:
            self.control.publish_data( "Try to survey find path")
            if( self.mission_path.find_path() ):
                self.control.publish_data( "Good luck You finish second path")
            else:
                self.control.publish_data( "It bad you failure mission path rotation to drop" )
                self.control.relative_yaw( STRATEGY_ROTATION_BUOY_DROP )

        # Start part for search drop garliac mission
        self.control.activate( ['x' , 'y'] )
        self.control.sleep()

        self.control.absolute_z( STRATEGY_DEPTH_FIND_DROP )
        self.control.sleep()
        self.control.publish_data( "STRATEGY Command depth at " 
            + str( STRATEGY_DEPTH_FIND_DROP - 0.5)  )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.control.absolute_z( STRATEGY_DEPTH_FIND_DROP )
        self.control.publish_data( "STRATEGY Command depth at " + str(STRATEGY_DEPTH_FIND_DROP))
        self.control.publish_data( "Waiting yaw")
        while( not self.control.check_yaw( 0.12 ) ):
            self.rate.sleep()

        self.control.deactivate( ['x' , 'y'] )
        self.control.force_xy( STRATEGY_FORCE_DROP , 0 )

        # Move free move
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        self.control.publish_data( "STRATEGY Move free time for drop")
        while( (not rospy.is_shutdown() ) and diff_time < STRATEGY_FREE_TIME_DROP ):
            self.rate.sleep()
            self.control.force_xy( STRATEGY_FREE_FORCE_DROP , 0 )
            self.control.publish_data( "STRATEGY Move free time is " 
                + repr( ( diff_time , STRATEGY_FREE_TIME_DROP ) ) )
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        self.control.publish_data( "START FORWARD TO FIND DROP MISSION" )
        count_found = 0
        self.control.force_xy( STRATEGY_FORCE_DROP , 0 , True )
        while( ( not rospy.is_shutdown() ) and diff_time < STRATEGY_TIME_DROP ):
            self.rate.sleep()
            self.vision_drop.call_data( DROP_FIND_TARGET )
            self.vision_drop.echo_data()
            if( self.vision_drop.result['found'] ):
                count_found += 1
                if( count_found == 5 ):
                    self.control.publish_data( "STRATEGY Focuse on target")
                    target_depth = STRATEGY_DEPTH_FIND_DROP
                    self.control.force_xy( 0 , 0 )                
                    while( ( not rospy.is_shutdown() ) and count_found > 0 ):
                        self.rate.sleep()
                        self.vision_drop.call_data( DROP_FIND_TARGET )
                        self.vision_drop.echo_data()
                        if( self.vision_drop.result['found'] ):
                            count_found = 5
                        else:
                            count_found -= 1
                            self.control.publish_data( "Don't found picture " 
                                + str( count_found ) )
                            continue
                        force_x = 0 
                        force_y = 0
                        if( ( abs( self.vision_drop.result['center_x'] ) > 15 ) or
                                ( abs( self.vision_drop.result['center_y'] ) > 15 ) ):

                            if( self.vision_drop.result['center_y'] <  -15 ):
                                force_x = TARGET_BACKWARD
                            elif( self.vision_drop.result['center_y'] > 15 ):
                                force_x = TARGET_FORWARD
                            else:
                                pass

                            if( self.vision_drop.result['center_x'] > 15 ):
                                force_y = SURVEY_RIGHT
                            elif( self.vision_drop.result['center_x'] < -15 ):
                                force_y = SURVEY_LEFT
                            else:
                                pass

                            self.control.publish_data("Command force {:6.3f} , {:6.3f}".format(
                                force_x , force_y ) ) 
                            self.control.force_xy( force_x , force_y )
                        else:
                            if( self.control.check_z( 0.12 ) ):
                                if( target_depth > DROP_START_DEPTH ):
                                    self.control.publish_data( "Command to depth at " 
                                        + str( DROP_START_DEPTH ) )
                                    self.control.absolute_z( DROP_START_DEPTH )
                                    target_depth = DROP_START_DEPTH
                                else:
                                    self.control.publish_data( "Depth is ok")
                                    break
                            else:
                                self.control.force_xy( 0 , 0 )
                                self.control.publish_data( "Now center waiting detph")
                    break
            else:
                count_found = 0
            distance = self.control.force_xy( STRATEGY_FORCE_DROP , 0 ) 
            diff_time = (rospy.get_rostime() - start_time).to_sec()
            self.control.publish_data( 
                "time ( {:6.3f} , {:6.3f} ) and distance ( {:6.3f} , {:6.3f} ) found ".format(
                    diff_time , STRATEGY_TIME_DROP , distance , STRATEGY_DISTANCE_DROP ) 
                + str( count_found ) )
            if( distance > STRATEGY_DISTANCE_DROP ):
                self.control.publish_data( "Abort to find drop by distance")
                break

        self.control.activate( ['x' , 'y' ] )
        self.control.relative_xy( 0 , 0 )

        if( count_found > 0 ):
            self.control.publish_data( "Found picture next play drop by operator function")
            self.mission_drop.operator()
        else:
            self.control.publish_data( "Don't found drop" )

        if STRATEGY_CHOICE_PROCESS == 0 :
            self.control.publish_data( "STRATEGY to choose process last mission by don't use dvl")
            self.not_use_dvl()
        elif STRATEGY_CHOICE_PROCESS == 1 :
            self.control.publish_data( "STRATEGY to choose process last mission by use DVL")
            self.use_dvl()
        else:
            self.control.publish_data( "STRATEGY to choose process last missiob by use pinger")

    # End part of play all mission

        self.control.publish_data( "Finish all strategy mission" )
        self.control.deactivate( ("x", "y", "z", "roll", "pitch", "yaw") )

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

    def use_dvl( self ):
        self.control.publish_data( "STRATEGY use DVL start mission by target at exposed")

        self.control.update_target()
        collective_target_yaw = zeabus_math.bound_radian(
            self.control.target[ 5 ] + STRATEGY_ROTATION_STAKE )

        self.control.absolute_z( STRATEGY_EXPOSED_FIND )
        self.control.publish_data( "STRATEGY command depth to " + str( STRATEGY_EXPOSED_FIND ) )
        self.control.sleep()

        self.control.relative_yaw( STRATEGY_ROTATION_EXPOSED )
        self.control.publish_data( "STRATEGY rotation for find target coffin to exposed " + str( 
            STRATEGY_ROTATION_EXPOSED ) )
        self.control.sleep()

        while not self.control.check_yaw( 0.15 ):
            self.rate.sleep()

        self.control.publish_data( "STRATEGY distance survey is " + str( STRATEGY_DISTANCE_SURVEY ) )
        self.control.relative_xy( 0 , STRATEGY_DISTANCE_SURVEY )
        self.control.sleep()

        while not self.control.check_xy( 0.15 , 0.15 ):
            self.rate.sleep()

        self.control.publish_data( "STRATEGY distance forward is " + str(STRATEGY_DISTANCE_FORWARD) )
        self.control.relative_xy( STRATEGY_DISTANCE_FORWARD , 0 )
        self.control.update_target()
        collective_point = ( self.control.target_pose[0] , self.control.target_pose[1] )
        play_mission = False 
        count_found = 0
        while not self.control.check_xy( 0.15 , 0.15 ) :
            self.rate.sleep()
            self.vision_coffin.call_data()
            self.vision_coffin.echo_data()
            if self.vision_coffin.result['num_object'] > 0 :
                count_found += 1            
                if count_found == 3:
                    self.control.publish_data( "STRATEGY Find coffind stop survey and make center")
                    self.control.deactivate( ('x' , 'y' ) )
                    count_unfound = 0 
                    while ( not rospy.is_shutdown() ) and count_unfound < 3 :
                        self.rate.sleep()
                        self.vision_coffin.call_data()
                        self.vision_coffin.echo_data()
                        if self.vision_coffin.result['num_object'] > 0 :
                            count_unfound = 0
                            force_x = 0 
                            force_y = 0 
                            ok_x = False
                            ok_y = False
                            if self.vision_coffin.result['center_x'] < -20 :
                                force_y = SURVEY_LEFT
                            elif self.vision_coffin.result['center_x' ] > 20 :
                                force_y = SURVEY_RIGHT
                            else:
                                ok_y = True

                            if self.vision_coffin.result['center_y'] < -20 :
                                force_x = SURVEY_BACKWARD
                            elif self.vision_coffin.result['center_y'] > 20 :
                                force_x = SURVEY_FORWARD
                            else:
                                ok_x = True
                            if ok_x and ok_y :
                                count_found = 3
                                self.control.publish_data( "STRATEGY now center point of coffin")
                                self.control.force_xy( 0 , 0 )
                                break
                            else:
                                self.control.force_xy( force_x , force_y )
                                self.control.publish_data( "STRATEGY command force " 
                                    + repr( (force_x , force_y ) ) )    
                        else:
                            count_unfound += 1
                            count_found = 0
                    self.control.activate( ('x' , 'y' ) )
                    if count_unfound == 3 :
                        self.control.publish_data( "STRATEGY think target it is noise continue")
                        self.control.absolute_xy( collective_point[0] , collective_point[1] )
                        self.control.sleep() 
                    else:
                        break
                else:
                    self.control.publish_data( "STRATEGY found coffin object is " 
                        + str( count_found ))
            else:
                count_found = 0

        result = False
        if count_found == 3 :
            self.mission_exposed.operator()
            result = True
        else:
            result = self.mission_exposed.find()

        if result:
            self.control.publish_data( "STRATEGY finish play exposed consider try to play stake")
            self.control.absolute_yaw( collective_target_yaw )
            self.control.publish_data( "STRATEGY command target yaw is " 
                + str( collective_target_yaw ) )
            self.control.absolute_z( STRATEGY_STAKE_DEPTH )
            self.control.publish_data( "STRATEGY command depth to " 
                + str( STRATEGY_STAKE_DEPTH ) )
            self.control.sleep()

            while not self.control.check_yaw( 0.15 ):
                self.rate.sleep()

            self.control.relative_xy( 0 , STRATEGY_STAKE_DISTANCE_SURVEY )
            self.control.publish_data( "STRATEGY command survey is " 
                + str( STRATEGY_STAKE_DISTANCE_SURVEY ) )
            self.control.sleep()
            while not self.control.check_xy( 0.15 ):
                self.rate.sleep()

            self.control.relative_xy( STRATEGY_STAKE_DISTANCE_FORWARD , 0 )
            self.control.publish_data( "STRATEGY command forward is " 
                + str( STRATEGY_STAKE_DISTANCE_FORWARD ) )
            self.control.sleep()

            self.control.update_target()
            collective_point = ( self.control.target_pose[0] , self.control.target_pose[1] )

            count_found = 0
            while ( not self.control.check_xy( 0.15 ) ) and count_found < 3 :
                self.rate.sleep()
                self.vision_stake.call_data( STAKE_FIND_TARGET )
                self.vision_stake.echo_data()

                if self.vision_stake.result['found'] :
                    count_found += 1
                    if count_found == 3 :
                        self.control.publish_data( "STRATEGY Now found target stop and focus")
                        count_unfound = 0
                        self.control.deactivate( ('x' , 'y' ) )
                        while ( not rospy.is_shutdown() ) and count_unfound < 3 :
                            self.rate.sleep()
                            self.vision_stake.call_data( STAKE_FIND_TARGET )
                            self.vision_stake.echo_data()
                            if self.vision.result['found'] :
                                count_unfound = 0
                                force_x = 0
                                force_y = 0
                                ok_x = False
                                ok_z = False
                                if self.vision.result['center'][0] > 10 :
                                    force_y = TARGET_LEFT
                                elif self.vision.result['center'][0] < -10 :
                                    force_y = TARGET_RIGHT
                                else:
                                    ok_x = True

                                if self.vision.result['area'] < STRATEGY_STAKE_AREA_FOCUS / 1.5
                                    force_x = TARGET_FORWARD
                                else:
                                    ok_z = True
                                
                                if ok_x and ok_z :
                                    count_found = 3 
                                    self.control.force_xy( 0 , 0 )
                                    self.mission_stake.operator()
                                    break
                                else:
                                    self.control.force_xy( force_x , force_y )
                                    self.control.publish_data("STRATEGY command force is " + repr(
                                        force_x , force_y ) )
                            else:
                                count_unfound += 1
                        self.control.activate( ('x' , 'y' ) )
                        if count_unfound == 3:
                            self.control.publish_data( "STRATEGY target disapper" )
                            self.control.absolute_xy( collective_point[0] , collective_point[1] )
                            self.control.sleep()
                        else:
                            self.control.publish_data( "STRATEGY Give process to stake")
                    else:
                        self.control.publish_data( "STRATEGY found target " + str( count_found ) )
                else:
                    count_found = 0
        else:
            self.control.publish_data( "STRATEGY don't continue to play because exposed false")

            

    def not_use_dvl( self ):

        self.control.publish_data( "STRATEGY not use DVL start mission by target at exposed")

        self.control.update_target()
        collective_target_yaw = zeabus_math.bound_radian( 
            self.control.target[5] + STRATEGY_ROTATION_STAKE )

        self.control.absolute_z( STRATEGY_EXPOSED_FIND )
        self.control.publish_data( "STRATEGY command depth to " + str( STRATEGY_EXPOSED_FIND ) )
        self.control.sleep()
        
        self.control.relative_yaw( STRATEGY_ROTATION_EXPOSED )
        self.control.publish_data( "STRATEGY rotation for find target coffin to exposed " + str(
            STRATEGY_ROTATION_EXPOSED ) )
        self.control.sleep()

        while not self.control.check_yaw( 0.12 ):
            self.rate.sleep()

        while not self.control.check_z( 0.12 ):
            self.rate.sleep()

        self.control.deactivate( ('x', 'y') )
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while not rospy.is_shutdown() and diff_time < STRATEGY_TIME_SURVEY: 
            self.rate.sleep()
            self.control.force_xy( 0 , STRATEGY_FORCE_SURVEY )
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "STRATEGY Survey for expose on time " + str( diff_time ) )
        self.control.force_xy( 0 , 0 ) 

        self.control.activate( ('x' , 'y' ) )

        self.control.publish_data( "STRATEGY Waiting yaw before forward")
        while not self.control.check_yaw( 0.12 ) :
            self.rate.sleep()

        self.control.deactivate( ('x' , 'y') )

        self.control.publish_data( "STRATEGY forward to find exposed" )
        count_found = 0
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while (not rospy.is_shutdown()) and diff_time < STRATEGY_TIME_FORWARD and count_found < 5 : 
            self.rate.sleep()
            self.vision_coffin.call_data()
            self.vision_coffin.echo_data()
            if self.vision.result['num_object'] > 0 :
                count_found += 1
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False
                self.control.publish_data( "STRATEGY found target command force "
                    + repr( (force_x , force_y ) ) )
                if self.vision_coffin.result['center_x'] < -20 :
                    force_y = SURVER_LEFT
                elif self.vision_coffin.result['center_x']  > 20 :
                    force_y = SURVEY_RIGHT
                else:
                    ok_y = True
            
                if( self.vision_coffin.result['center_y'] < -20 ):
                    force_x = SURVEY_BACKWARD
                elif self.vision_coffin.result['center_y'] > 20:
                    force_x = SURVEY_FORWARD
                else:
                    ok_x = True 

                if( ok_x and ok_y ):
                    count_found = 5
                    self.control.publish_data( "STRATEGY now center point of coffin")
                else:
                    self.control.force_xy( force_x , force_y )
                    self.control.publish_data( "STRATEGY Have picture command force " + repr( (
                        force_x , force_y ) ) )
            else:
                count_found = 0
                self.control.force_xy( SURVEY_FORWARD , 0 )
                self.control.publish_data( "STRATEGY Don't found picture command survey forward" )
            diff_time = (rospy.get_rostime() - start_time).to_sec()

        self.control.activate( [ 'x' , 'y' ] )
        result = False        
        if( count_found == 5 ):
            self.control.publish_data( "STRATEGY finish and found object let operator")
            self.mission_exposed.operator()
            result = True
        else:
            self.mission_exposed.publish_data( "STRATEGY finish forward search let find object")
            result = self.mission_exposed.find()

        if result :
            self.control.publish_data( "STRATEGY success mission I will try to continue stake" )
            self.control.absolute_yaw( collective_target_yaw )
            self.control.publish_data( "STRATEGY command absolute yaw " 
                + str( collective_target_yaw ) )
            self.control.sleep()
            while not self.control.check_yaw( 0.12 ):
                self.rate.sleep()

            self.control.absolute_z( STRATEGY_STAKE_DEPTH )
            self.control.publish_data( "STRATEGY command absolute depth is " 
                + str( STRATEGY_STAKE_DEPTH ) )

            self.control.deactivate( ('x' , 'y' ) )            
            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "STRATEGY survey before try to forward find stake mission")
            while ( not rospy.is_shutdown() ) and diff_time < STRATEGY_STAKE_TIME_SURVEY :
                self.rate.sleep()
                self.control.force_xy( 0.0 , STRATEGY_STAKE_FORCE_SURVEY )
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.control.publish_data( "STRATEGY Survey to stake on time " + str(diff_time) )

            self.control.activate( ('x' , 'y') )
            self.control.force_false()
            self.control.publish_data( "STRATEGY waiting yaw")
            while not self.control.check_yaw( 0.15 ):
                self.rate.sleep()

            self.control.deactivate( ( 'x' , 'y' ) )
            stat_time = rospy.get_rostime()
            diff_time = 0
            count_found = 0
            while ( not rospy.is_shutdown() ) and diff_time < STRATEGY_STAKE_TIME_FORWARD :
                self.rate.sleep()
                self.control.force_xy( STRATEGY_TIME_FORWARD , 0.0 )
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.vision_stake.call_data()
                self.vision_stake.echo_data()             
                
                if self.vision_stake.result['found']:
                    count_found += 1
                    if count_found == 3 :
                        self.control.publish_data( "STRATEGY find stake I will target on that")
                        count_unfound = 0
                        self.control.force_xy( 0 , 0 )
                        while not rospy.is_shutdown() and count_unfound < 3 :
                            self.rate.sleep()
                            self.vision_stake.call_data( STAKE_FIND_TARGET )
                            self.vision_state.echo_data()
                            force_x = 0 
                            force_y = 0
                            ok_x = False
                            ok_y = False
                            if self.vision_state.result['found'] :
                                count_unfound = 0
                                if self.vision_stake.result['center'][0] > 20 :
                                    force_y = TARGET_RIGHT
                                elif self.vision_stake.result['center'][1] < -20 :
                                    force_y = TARGET_LEFT
                                else:
                                    ok_y = True

                                if self.vision.result['area'] < STRATEGY_STAKE_AREA_FOCUS :
                                    force_x = TARGET_FORWARD
                                else:
                                    ok_x = True
                                if ok_x and ok_y :
                                    self.control.force_xy(  0 , 0 )
                                    self.control.publish_data( "STRATEGY get process to stake")
                                    self.mission_stake.operator()
                                    break
                                else:
                                    self.control.publish_data( "STRATEGY focuse target " 
                                        + repr( ( force_x , force_y ) ) )
                            else:
                                count_unfound += 1
                                self.control.publish_data( "STRATEGY Don't found target " 
                                    + str( count_unfound ) )
                                self.control.force_xy( 0 , 0 )
                        if count_unfound == 3:
                            self.control.publish_data( "Strategy Continue until time out")
                            count_found = 0
                        else:
                            self.control.publish_data( "STRATEGY Finish play break by after play")
                            break
                else:
                    count_found = 0
            
                self.control.publish_data( "STRATEGY count found " + str( count_found ) )
        else:
            self.control.publish_data( "STRATEGY faliure mission exposed don't play stake" )

        self.control.publish_data( "STRATEGY finish function play not_use_dvl")

if __name__=="__main__":
    rospy.init_node('strategy_mission')

    mission = StrategySpeed()

