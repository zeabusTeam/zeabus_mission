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
    
        self.vision = AnalysisGate( "base_gate" )
        self.control = CommandInterfaces( "GATE" )

        self.rate = rospy.Rate( 5 )

        self.state_mission = 0 

        self.save_point_x = 0
        self.save_point_y = 0
        

    def start_mission( self ): # state mission is 0

        self.control.publish_data( "START_MISSION doing mission gate")

        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -0.8 )

        self.control.publish_data( "START_MISSION waiting ok depth")
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.start_constant()

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
                self.control.publish_data( "START_CONSTANT Found picture")
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
                self.control.publish_data( "START_CONSTANT Found picture")
                count += 1
            else:
                count = 0

        if( count == 3 ):
            temp_relative_x = 0
            temp_relative_y = ( self.vision.result[ 'center_x' ] * 2.0 )/ -100
            if( self.vision.result[ 'distance' ] > 4 ):
                temp_relative_x = 1
            else:
                temp_relative_x = self.vision.result['distance'] / 2
            
            self.control.relative_xy( temp_relative_x , temp_relative_y )
            self.control.publish_data( "START_CONSTANT I will move " + str( temp_relative_x ) 
                + " and move y  " + str( temp_relative_y ) )
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
        while( not self.control.check_xy( 0.15  , 0.15 ) ):
            self.rate.sleep()
    
        self.control.publish_data( "LOCK_TARGET waiting yaw are ok")
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()
    
        self.control.deactivate( [ 'x' , 'y' ] )

        count_unfound = 0
        self.last_distance_to_found = 0

        while( not rospy.is_shutdown() ):
            self.vision.call_data()
            self.vision.echo_data() 

            # unit /100
            relative_x = 0
            relative_y = 0
            if( self.vision.result['found'] ):
                self.last_distance_to_found = self.vision.result[ 'distance' ]
                count_unfound = 0
                if( self.vision.result['center_x'] < -20 ):
                    relative_y = GATE_FORCE_Y
                elif( self.vision.result[ 'center_x'] > 20 ):
                    relative_y = -1.0*GATE_FORCE_Y
                else:
                    relative_x = GATE_FORCE_X

                if( self.vision.result[ 'distance' ] < 2.5 ):
                    self.control.publish_data( "LOCK_TARGET near target I break now" )
                    self.rate.sleep()
                    break
            else:
                count_unfound += 1
                self.control.publish_data( "LOCK_TARGET count_unfound " + str( count_unfound ) )
                relative_x = 200

            self.force_interfaces( relative_x , relative_y )
            if( count_unfound == 3 ):
                self.control.publish_data( "LOCK_TARGET breaking down don't found gate" )
                break

        self.control.activate( ['x' , 'y'])
        self.control.publish_data( "LOCK_TARGET to front gate {:4.2f}".format( 
            self.last_distance_to_found ) )
        if( self.last_distance_to_found < 2.2 ):
            self.control.publish_data( "LOCK_TARGET Don't move last because it near")
        else:
            self.control.publish_data( "LOCK_TARGET Move " 
                + str( self.last_distance_to_found - 1 ) )
            self.control.relative_xy( self.last_distance_to_found - 1.5 , 0 )

    def force_interfaces( self , x , y ): # Use for print abotu when force have change 
        if( ( self.save_point_x != 0 or self.save_point_y != y ) ):
            self.control.publish_data("LOCK_TARGET command force ( {:6.3f} , {:6.3f})".format(
                x , y ) )
            self.save_point_x = x
            self.save_point_y = y
        self.control.force_xy( 1.0*x/100 , 1.0*y/100 )

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
