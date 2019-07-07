#!/usr/bin/env python2
# FILE			: gate.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 27 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from __future__ import print_function
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_gate import AnalysisGate
from mission_constant import *

import rospy

class Gate:

    def __init__( self ):
    
        self.vision = AnalysisPath( "base_gate" )
        self.control = CommandInterfaces( "GATE" )

        self.rate = rospy.Rate( 5 )

        self.state_mission = 0 

        self.save_point_x = 0
        self.save_point_y = 0
        

    def start_mission( self ): # state mission is 0

        self.control.publish_data( "START_MISSION doing mission gate")

        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -1 )

        self.control.publish_data( "START_MISSION waiting ok depth")
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

    def start_constant( self ):
        self.control.publish_data( "START_CONSTANT survey to find mission by survey" )

        self.control.publish_data( "START_CONSTANT Waiting yaw are ok")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "START_CONSTANT Waiting xy are ok")
        while( not self.control.check_xy( 0.15 , 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "START_CONSTANT start forward " + str( GATE_START_FORWARD ) )
        self.control.relative_xy( GATE_START_FORWARD , 0 )
        self.rate.sleep()
        count = 0 
        while( ( not self.control.check_xy( 0.15 , 0.15 ) ) and count < 3):
            self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count += 1
            else:
                count = 0

        if( count == 3 ):
            self.control.relative_xy( 0 , 0 )
            self.control.publish_data( "START_CONSTANT Don't move I find target")
        else:
            self.control.publish_data( "START_CONSTANT start survey " + str( GATE_START_SURVEY) )
            self.control.relative_xy( 0 , GATE_START_SURVEY )
            count = 0 

        while( ( not self.control.check_xy( 0.15 , 0.15 ) ) and count < 3 ):
            self.rate.sleep()
    
            self.vision.call_data()
            self.vision.echo_data()
        
            if( self.vision.result['found'] ):
                count += 1
            else:
                count = 0

        if( count == 3 ):
            self.control.relative_xy( 0 , ( self.vision.result[ 'center_x' ] * 1.5 )/ -100 )
            self.control.publish_data( "START_CONSTANT Don't survey I find target" )
            self.lock_target()
        else:
            self.direct_search()

        
    def direct_search( self ):
        self.control.publish_data( "DIRECT_SEARCH start to search" )

        self.control.publish_data( "DIRECT_SEARCH waiting yaw" )
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.control.publish_data( "DIRECT_SEARCH waitign xy" )
        while( not self.control.check_xy( 0.15 , 0.15 ) ):     
            self.rate.sleep()

        self.control.publish_data( "COMMAND TO MOVE DIRECT TO GATE " 
            + str( GATE_LIMIT_DISTANCE ) + " meters" )

        count = 0
        while( ( not self.control.check_xy( 0.15 , 0.15 ) ) and ( count < 3 ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result[ 'found' ] ):
                count += 1
            else:
                count = 0

        if( count == 3 ):
            self.control.publish_data( "We found the picture so I will reset state")
            self.control.reset_state( 0 , 0)
            self.lock_target()
        else:
            self.control.publish_data( "We don't found picture abort that" )


    def lock_target( self ):
        self.control.publish_data( "LOCK_TARGET start mission")

        self.control.publish_data( "LOCK_TARGET waiting xy are ok")
        while( not self.control.check_xy( 0.15 , 0.15 ) ):
            self.rate.sleep()
    
        self.control.publish_data( "LOCK_TARGET waiting yaw are ok")
        while( not self.control.check_yaw( 0.15 , 0.15 ) ):
            self.rate.sleep()
    
        self.control.deactivate( [ 'x' , 'y' ] )

        count_unfound = 0
        last_distance_to_found = 0

        while( not rospy.is_shutdown() ):
            self.vision.call_data()
            self.vision.echo_data() 

            # unit /100
            relative_x = 0
            relative_y = 0
            if( self.vision.result['found'] ):
                last_distance_to_found = self.vision.result[ 'distance' ]
                count_unfound = 0
                if( self.vision.result['center_x'] < -20 ):
                    relative_y = 60
                elif( self.vision.result[ 'center_x'] > 20 ):
                    relative_y = -60
                else:
                    relative_x = 100

                if( self.vision.result[ 'distance' ] < 4 ):
                    last_distance_to_found = 0
                    self.control.publish_data( "LOCK_TARGET near target I break now" )
                    self.rate.sleep()
                    break
            else:
                count_unfound += 1
                relative_x = 100

            self.force_interfaces( relative_x , relative_y )
            if( count_unfound == 3 ):
                self.control.publish_data( "LOCK_TARGET breaking down don't found gate" )

        self.control.activate( ['x' , 'y'])
        final_distance = ( last_distance_to_found - 4 ) + 2
        self.control.publish_data( "LOCK_TARGET to front gate {:4.2f}".format( final_distance ) )

        self.control.relative_xy( final_distance )
        

    def force_interfaces( self , x , y ): # Use for print abotu when force have change 
        if( ( self.save_point_x != 0 or self.save_point_y != y ) ):
            self.control.publish_data("LOCK_TARGET command force ( {:6.3f} , {:6.3f})".format(
                x , y ) )
            self.save_point_x = x
            self.save_point_y = y
        self.control.force( 1.0*x/100 , 1.0*y/100 ) 
