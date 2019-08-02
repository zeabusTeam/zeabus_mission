#!/usr/bin/env python2
# FILE			: buoy_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 05 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_buoy import AnalysisBuoy
from constant_speed import *

class Buoy:

    def __init__( self ):

        self.vision = AnalysisBuoy( "base_buoy" )
        self.control = CommandInterfaces( "BUOY" )

        self.rate = rospy.Rate( 5 )

        self.status_mission = 0

        self.ok_count = 5 


    def start_mission( self ): # status_mission is 0

        self.control.reset_state()

        self.control.publish_data( "START Waiting z depth")
        self.control.absolute_z( BUOY_START_DEPTH )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "START Waiting yaw ok")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "Start mision find target")
        self.vision.call_data()
        self.vision.echo_data()
        if( self.vision.result['found'] ):
            temp_y = self.vision.result['center_x'] * -2 / 100
            self.control.publish_data( "Find target change to lock target and survey " 
                + str(temp_y))
            self.control.relative_xy( 2 , temp_y )
            self.lock_target()
        else:
            self.find_target()

    def find_target( self ): # status_mission = 1
        # Please thinking about plus sign to estimate value
        self.control.publish_data( "Welcome to mode find_target" )

        self.control.publish_data( "Find target Have to waiting depth before go to find target")

        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        relative_y = -0.8
        relative_x = 1

        self.control.publish_data( "Find target move relative y " + str( relative_y ) )
        self.control.relative_xy( 0 , relative_y )
        self.control.sleep()
        relative_y *= -2
        # mode 0 is forward , mode 1 is survey
        mode = 1

        while( not rospy.is_shutdown() ):
            self.rate.sleep()

            count = 0
            while( (not rospy.is_shutdown()) and (count < BUOY_FOUND_PICTURE ) ):
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.result['found'] ):
                    self.control.publish_data("FIND_TARGET Found picture")
                    count += 1
                else:
                    self.control.publish_data("FIND_TARGET don't found picture")
                    break

            if( count == BUOY_FOUND_PICTURE ):
                self.control.publish_data( "Find target Found 2 round move to mode lock_target")
                break

            if( not ( self.control.check_xy(0.15 , 0.15 ) and self.control.check_yaw( 0.15 ) )):
                continue

            if( mode == 0 ):
                self.control.publish_data( "Find target forward " +str( relative_x ) +" meter" )
                self.control.relative_xy( relative_x , 0 )
                self.control.sleep()
                mode = 1
            else:
                mode = 0
                self.control.publish_data( "Find target survey " + str( relative_y ) + " meter" )
                self.control.relative_xy( 0 , relative_y )
                self.control.sleep()
                relative_y *= -1

        self.control.publish_data( "FIND_TARGET reset position xy")
        self.control.relative_xy( 0 , 0 )
 
        self.lock_target()

    def lock_target( self ): # status_mission = 2

        self.control.publish_data( "Welcome to mode lock target" )

        unfound = 0 

        distance = 3
        warning_time = 5

        time_out = BUOY_TIME_LOCK_TARGET # time_out seconds when you have lock target
        self.control.publish_data( "You have limit time in mode lock_target " + str( time_out ))

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()

        self.control.deactivate( ["x" , "y"] )

        limit_time = BUOY_LIMIT_TIME
        
        while( ( not rospy.is_shutdown() ) and ( diff_time < time_out ) ):

            self.control.publish_data( "Lock Target mode lock_target call vision")
            self.vision.call_data()
            self.vision.echo_data()

            relative_x = 0
            relative_y = 0
            if( self.vision.result['found'] ):
                unfound = 0
                if( self.vision.result['center_x'] > 15 ):
                    relative_y = TARGET_RIGHT
                elif( self.vision.result['center_x'] < -15 ):
                    relative_y = TARGET_LEFT
                else:
                    relative_x = SUPER_FORWARD

                self.control.publish_data( "Lock Target command force ({:4.2f},{:4.2f})".format(
                    relative_x , relative_y ) )
                self.control.force_xy( relative_x , relative_y )

                if( self.vision.result['area'] > BUOY_AREA_ABORT ):
                    self.control.publish_data( "Break from area condition") 
                    break

                limit_time = self.vision.result['distance'] * 4

            else:
                unfound += 1
                self.control.publish_data( "Don't found target" )
                if( unfound == 2 ):
                    self.control.publish_data( "Move to dash_mode")
                    break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            if( ( diff_time / warning_time ) > 1 ):
                self.control.publish_data( "Warning over {:6.2f} limit at {:6.2f}".format(
                    warning_time , time_out ) )
                warning_time += 5 

        self.dash_mode( distance , limit_time)

        self.control.force_xy( 0 , 0 )
        self.rate.sleep()

        self.control.publish_data( "Finish dash mode move back")
        self.finish_task()
        
        self.control.activate( ["x" , "y"] )

    def dash_mode( self , distance , limit_time = BUOY_LIMIT_TIME ):

        self.control.force_xy( 1 , 0 , True)
        start_time = rospy.get_rostime()
        self.control.publish_data( "DASH Move forward limit time at " + str( limit_time ) )
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( ( not rospy.is_shutdown() ) and ( diff_time < limit_time ) ):
            self.rate.sleep()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            temp = self.control.force_xy( SUPER_FORWARD , 0 )
            self.control.publish_data( "DASH Now distance is " + str( temp ) 
                + " and time is " + str( diff_time ) )

    def finish_task( self ):
        self.rate.sleep()
        self.control.publish_data( "FINISH this task, Move back")
        self.control.absolute_z( BUOY_TARGET_DEPTH_FINISH )

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( (not rospy.is_shutdown() ) and diff_time < BUOY_TIME_TO_BACK ):
            self.rate.sleep()
            self.control.force_xy( -1.0*BUOY_FORCE_FORWARD , 0 )
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "FINISH back current , limit " 
                + repr( ( diff_time , BUOY_TIME_TO_BACK ) ) )

        self.control.publish_data( "FINISH Waiting depth")
        while( not self.control.check_z( 0.15 ) ):
            self.control.force_xy( 0 , 0 )
            self.rate.sleep()

        self.control.publish_data( "FINISH Survey right")
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( (not rospy.is_shutdown() ) and diff_time < BUOY_TIME_TO_SURVEY ):
            self.rate.sleep()
            self.control.force_xy( 0 , BUOY_FORCE_SURVEY )
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "FINISH survey current , limit " 
                + repr( ( diff_time , BUOY_TIME_TO_SURVEY ) ) )
           
        self.control.publish_data( "FINISH Forward")
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( (not rospy.is_shutdown() ) and diff_time < ( BUOY_TIME_TO_BACK + 5 ) ):
            self.rate.sleep()
            self.control.force_xy( BUOY_FORCE_FORWARD , 0 )
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.publish_data( "FINISH forward current , limit " 
                + repr( ( diff_time , BUOY_TIME_TO_BACK + 5 ) ) )

        self.control.force_xy( 0 , 0 )
        self.control.publish_data( "Finish task buoy?")

if __name__=="__main__" :
    rospy.init_node( "mission_buoy" )

    mission = Buoy()
    mission.start_mission() 
