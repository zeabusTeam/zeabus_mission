#!/usr/bin/env python2
# FILE			: gate_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_gate import AnalysisGate
from constant_speed import *

import rospy

class Gate:

    def __init__( self ):
    
        self.vision = AnalysisGate( "base_gate" )
        self.control = CommandInterfaces( "GATE" )

        self.rate = rospy.Rate( 5 )

        self.state_mission = 0 

        self.save_point_x = 0
        self.save_point_y = 0
        self.last_distance_to_found = 0

    def start_mission( self ): # state mission is 0

        self.control.publish_data( "START doing mission gate")

        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( GATE_START_DEPTH )

        self.control.publish_data( "START survey to find mission by survey" )

        self.control.publish_data( "START Waiting yaw are ok")

        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "START DEACTIVATE XY and forward")
        self.control.deactivate( ['x' , 'y'] )

        # Start to forward by parameter and fix to move forward only
        self.control.force_xy( SURVEY_FORWARD , 0 , True )
        count = 0
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( diff_time < 4 ) and not rospy.is_shutdown() :
            self.rate.sleep()
            self.control.force_xy( SURVEY_FORWARD , 0 )
            self.control.publish_data("GATE START free forward")
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()  
        while( diff_time < GATE_START_FORWARD_TIME and count < 3 and not rospy.is_shutdown() ):
            self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count += 1
                self.control.publish_data( "START Found picture count is " + str( count ) )
                self.last_distance_to_found = self.vision.result['distance']
            else:
                count = 0

            if( self.control.force_xy( SURVEY_FORWARD , 0 ) > GATE_START_FORWARD_DISTANCE ):
                self.control.publish_data( "START Abort forward by distance")
                break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()  
            self.control.publish_data( "START forward time : limit " 
                + repr( (diff_time , GATE_START_FORWARD_TIME )  ) )

        if( count == 3 ):
            self.control.publish_data( "START Move to target picture")
            while( not rospy.is_shutdown() ):
                self.rate.sleep()
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.result['center_x'] < -30 ):
                    self.control.force_xy( 0.2 , SURVEY_LEFT )
                elif( self.vision.result[ 'center_x' ] > 30 ):
                    self.control.force_xy( 0.2 ,  SURVEY_RIGHT)
                else:
                    self.control.publish_data( "START Now center change to lock target")
                    break
        else:
            self.control.publish_data( "START survey to gate mission" )
            self.control.force_xy( 0 , SURVEY_LEFT * GATE_START_SURVEY_DIRECTION , True )
            count = 0 

        # Next If you don't found picture and count == 3 I will survey
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()  
        while( diff_time < GATE_START_SURVEY_TIME and count < 3 and not rospy.is_shutdown() ):
            self.rate.sleep()
    
            self.vision.call_data()
            self.vision.echo_data()
        
            if( self.vision.result['found'] ):
                self.control.publish_data( "START Found picture")
                count += 1
            else:
                count = 0

            if( self.control.force_xy( 0 , SURVEY_LEFT * GATE_START_SURVEY_DIRECTION ) 
                    > GATE_START_SURVEY_DISTANCE ):
                self.control.publish_data( "START Abort forward by distance")
                break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()  
            self.control.publish_data( "START survey time : limit " 
                + repr( ( diff_time , GATE_START_SURVEY_TIME )  ) )

        if( count == 3 ):
            self.control.publish_data( "START next function is lock_target" )
            self.lock_target()
        else:
            self.control.publish_data( "START next function is direct_search" )
            self.direct_search()

    # This function use to forward go to gate by don't found gate        
    def direct_search( self ):
        self.control.publish_data( "DIRECT_SEARCH start to search" )

        self.control.force_false()

        self.control.publish_data( "DIRECT_SEARCH waiting yaw" )
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        count = 0
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
        self.control.publish_data( "DIRECT_SEARCH start to forward")
        self.control.force_xy( SURVEY_FORWARD , 0 , True )
        while( ( diff_time < GATE_FORWARD_ONLY_TIME ) and ( count < 3 ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result[ 'found' ] ):
                count += 1
                self.last_distance_to_found = self.vision.result['distance']
                continue
            else:
                count = 0

            if( self.control.force_xy( SURVEY_FORWARD , 0 ) < GATE_FORWARD_ONLY_DISTANCE ):
                self.control.publish_data( "DIRECT_SEARCH abort by distance")
                break

        if( count == 3 ):
            self.control.force_false()
            self.lock_target()
        else:
            self.control.publish_data( "We don't found picture abort that" )


    def lock_target( self ):
        self.control.publish_data( "LOCK_TARGET start mission")

        count_unfound = 0

        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data() 

            # unit /100
            relative_x = 0
            relative_y = 0
            if( self.vision.result['found'] ):
                self.last_distance_to_found = self.vision.result[ 'distance' ]
                count_unfound = 0
                if( self.vision.result['center_x'] < -10 ):
                    relative_y = GATE_FORCE_Y
                elif( self.vision.result[ 'center_x'] > 10 ):
                    relative_y = -1.0 * GATE_FORCE_Y
                else:
                    relative_x = GATE_FORCE_X

                if( self.vision.result[ 'distance' ] < 2.5 ):
                    self.control.publish_data( "LOCK_TARGET near target I break now" )
                    self.rate.sleep()
                    break

                relative_z = 0
                if( self.vision.result['center_y'] < -50 ):
                    relative_z = -0.15
                elif( self.vision.result['center_y'] > -30 ):
                    relative_z = 0.15
                else:
                    relative_z = 0

#                if( ( relative_z != 0 ) and self.control.check_z( 0.15 ) ):
#                    self.control.publish_data( "LOCK_TARGET relative z : " + str( relative_z ) )

            else:
                count_unfound += 1
                self.control.publish_data( "LOCK_TARGET count_unfound " + str( count_unfound ) )
                relative_x = 200

            self.force_interfaces( relative_x , relative_y )
            if( count_unfound == 3 ):
                self.control.publish_data( "LOCK_TARGET breaking down don't found gate" )
                break

        self.control.force_false()
        self.control.publish_data( "LOCK_TARGET to front gate {:4.2f} and center is {:4.2f}".format( 
            self.last_distance_to_found , self.vision.result['center_x'] ) )
        # EDIT ABOUT LAST TIME TO PASS GATE
        if( self.last_distance_to_found > 2.5 ):
            self.control.publish_data( "LOCK_TARGET move for little forward")
            limit_time = ( self.last_distance_to_found - 2.0 ) * 5
            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            run_lock_target_again = False
            count_found = 0
            while( ( not rospy.is_shutdown() ) and diff_time < limit_time ):
                self.rate.sleep()
                self.control.force_xy( SUPER_FORWARD , 0 )
                self.control.publish_data( "LOCK_TARGET last forward time ( diff : limit ) "
                    +repr( ( diff_time , limit_time ) ) )
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.result['found'] ):
                    count_found += 1
                    self.control.publish_data("LOCK_TARGET run again? " + str( count_found ) )
                    if( count_found == 3 and GATE_APPROVE_AGAIN ):
                        run_lock_target_again = True
                        break
                else:
                    count_found = 0
            if( run_lock_target_again ):
                self.control.publish_data( "LOCK_TARGET run again" )
                self.lock_target()

        self.control.activate( ['x' , 'y'])

    def force_interfaces( self , x , y ): # Use for print abotu when force have change 
        if( ( self.save_point_x != 0 or self.save_point_y != y ) ):
            self.control.publish_data("LOCK_TARGET command force ( {:6.3f} , {:6.3f})".format(
                x , y ) )
            self.save_point_x = x
            self.save_point_y = y
        self.control.force_xy( 1.0*x/100 , 1.0*y/100 )


    # Below function is manual call
    def pass_target( self ):
        self.control.publish_data( "PASS_TARGET now is last step" )

        self.control.publish_data( "PASS_TARGET waiting xy ok" )
        while( not self.control.check_xy( 0.15 , 0.15 ) ):
            self.rate.sleep() 

        self.control.publish_data( "PASS_TARGET waiting yaw" )
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "PASS_TARGET move forward pass gate 4 meter"  )
        self.control.relative_xy( 4 , 0 )
        rospy.sleep( 0.2 )
        
        while( not self.control.check_xy( 0.15 , 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "PASS_TARGET Finish")

if __name__=="__main__" :
    rospy.init_node( "mission_gate" )

    mission = Gate()
    mission.start_mission()
    mission.pass_target()
