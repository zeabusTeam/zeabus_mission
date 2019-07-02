#!/usr/bin/env python2
# FILE			: buoy_straight.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 02 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   What is mean of name buoy_sraight
#       buoy is mission buoy. I think you can guess it.
#       straight is meaning I will found by striaght and survey left right to find target
#   Rule of Decision
#       Vision have send value about score of buoy in range 0 - 100
#           30 -> I will remember have this point 
#           50 -> I will go that to check picture
#           70 -> I beleive don't care other data
#   What process of this mode?
#       status = 0 
#           This function will use only for prepare depth to doing mission
#       status = 1
#           Survey to find mission and have check point but not survey left right.
#           I use spin and move to check instead
#       

# REFERENCE

import rospy
import math
from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_buoy import AnalysisBuoy

class Buoy:

    def __init__( self ):

        self.vision = AnalysisBuoy( "base_buoy" )
        self.control = CommandInterfaces( "BUOY" )

        self.rate = rospy.Rate( 5 )

        self.status_mission = 0

        self.ok_count = 5 


    def start_mission( self ): # status_mission is 0

        self.control.activate( ['x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw' ] )

        self.control.relative_xy( 0 , 0 )
        self.control.relative_z( 0 )
        self.control.relative_yaw( 0 )

        self.control.publish_data( " Waiting z depth")
        self.control.absolute_z( -2 )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( " Waiting xy ok")
        while( not self.control.check_xy( 0.15 , 0.15) ):
            self.rate.sleep()

        self.control.publish_data( "Waiting yaw ok")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "mode lock_target find target")
        self.vision.call_data()
        self.vision.echo_data()
        if( self.vision.result['found'] ):
            self.control.publish_data( "Find target move lock target")
            self.lock_target()
        else:
            self.find_target()



    def find_target( self ): # status_mission = 1
        # Please thinking about plus sign to estimate value
        self.control.publish_data( "Welcome to mode find_target" )

        self.control.publish_data( "Have to waiting depth before go to find target")

        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        relative_y = -0.8
        relative_x = 1

        self.control.publish_data( "move relative y " + str( relative_y ) )
        self.control.relative_xy( 0 , relative_y )
        relative_y *= -2
        # mode 0 is forward , mode 1 is survey
        mode = 0

        while( not rospy.is_shutdown() ):
            self.rate.sleep()

            while( self.control.check_xy( 0.15 , 0.15 ) ):
                self.rate.sleep()
            
            while( self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            count = 0
            while( not rospy.is_shutdown() ):
                self.vision.call_data()
                if( self.vision.result['found'] ):
                    count += 1
                else:
                    break

            if( count == 2 ):
                self.control.publish_data( "Found two round move to mode lock_target now")
                break

            if( mode == 0 ):
                self.control.publish_data( "Move forward " + str( relative_x ) + " meter" )
                self.control.relative_xy( relative_x , 0 )
                mode = 1
            else:
                mode = 0
                self.control.publish_data( "Move survey " + str( relative_y ) + " meter" )
                self.control.relative_xy( 0 , relative_y )
                relative_y *= -1
                 

        temp_y = self.vision.result['center_x'] / 100
        temp_x = 1 
        self.control.publish_data( "Move relative (x,y) : " + repr( ( temp_x , temp_y ) ) ) 
        self.control.relative_xy( temp_x , temp_y )

        if( self.vision.result[ 'center_y' ] > 80 ):
            self.control.publish_data( "Move up") 
            self.control.relative_z( 0.05 )
        elif( self.vision.result[ 'center_y'] < -80 ):
            self.control.publish_data( "Move dowm") 
            self.control.relative_z( -0.05)
        else:
            self.control.publish_data( "Don't move in axis z") 

        self.lock_target()

    def lock_target( self ): # status_mission = 2

        self.control.publish_data( "Welcome to mode lock target" )

        unfound = 0 

        distance = 3
        limit_time = 10
        warning_time = 10

        time_out = 30 # time_out seconds when you have lock target
        self.control.publish_data( "You have limit time in mode lock_target " + str( time_out ))

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()

        while( ( not rospy.is_shutdown() ) and ( diff_time < limit_time ) ):

            while( not self.control.check_z( 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( " Waiting xy ok")
            while( not self.control.check_xy( 0.15 , 0.15) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting yaw ok")
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( "mode lock_target find target")
            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result['found'] ):
                unfound = 0
                if( self.vision.result['center_x'] < -20 ):
                    self.control.publish_data( "Move left")
                    self.control.relative_xy( 0 , 0.3 )
                elif( self.vision.result['center_x'] > 20 ):
                    self.control.publish_data( "Move right")
                    self.control.relative_xy( 0 , -0.3 )
                else:
                    self.control.publish_data( "Move forward" )
                    self.control.relative_xy( 1.5 , 0)

                if( self.vision.result['area'] > 8 ):
                    self.control.publish_data( "Break from area condition") 
                    distance = 2
                    limit_time = 8.5
                    break 
                    

                if( self.vision.result['center_y'] > 70 ):
                    self.control.publish_data( "Move up")
                    self.control.relative_z( 0.1 )
                elif( self.vision.result['center_y'] < -70 ):
                    self.control.publish_data( "Move down" )
                    self.control.relative_z( -0.1 )
                else:
                    self.control.publish_data( "Don't move depth center_y : " 
                        + str( self.vision.result['center_y']) )
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
                warning_time += 10

        self.dash_mode( distance , limit_time )

    def dash_mode( self , distance , limit_time ):

        self.control.publish_data( "Waiting yaw before dash mode " + str(distance) )
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.deactivate( ["x" , "y"] )
        
        self.control.force_xy( 1 , 0 , True)
        start_time = rospy.get_rostime()
        self.control.publish_data( "Move forward limit time at " + str( limit_time ) )
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while(  not rospy.is_shutdown()  ):
            self.rate.sleep()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            temp = self.control.force_xy( 1 , 0 )
            self.control.publish_data( "Now distance is " + str( temp ) 
                + " and time is " + str( diff_time ) )
            if( temp > distance ):
                break
            elif( diff_time > limit_time ):
                break
        self.control.force_xy( 0 , 0 )
        self.control.activate( ["x" , "y"])

        self.control.publish_data( "Finish dash mode move back")
        self.finish_task()

    def finish_task( self ):
        self.control.relative_xy( -0.5 , -0.8 )
        self.control.publish_data( "Finish this task, Move back")
        while( not self.control.check_xy( 0.15 , 0.15 ) ):
            self.rate.sleep()

        self.control.absolute_z( -1 )
        self.control.publish_data( "Go to depth" )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "Finish task buoy?")

if __name__=="__main__" :
    rospy.init_node( "mission_buoy" )

    mission = Buoy()
    mission.start_mission() 
