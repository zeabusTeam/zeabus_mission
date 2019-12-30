#!/usr/bin/env python2
# FILE          : forward.py
# AUTHOR        : Supasan Komonlit
# CREATE ON     : 2019, December 30 (UTC+0)
# MAINTAINER    : K.Supasan

# README

# REFERENCE

import rospy

from std_msgs.msg import Bool

from zeabus.ros import message as nm

from zeabus.connection.control import ControlHandle 

START_TIME = 30

class Forward:

    def __init__( self ):

        rospy.init_node( "forward" )

        self.ch = ControlHandle( "forward" ) # Control Handle

        self.publish_state = rospy.Publisher( "/activate" , Bool , queue_size = 1 )

    def activate( self ):

        print( "Start count time before start 30 second" )

        self.ch.reset_all( True )

        self.ch.activate( False )
        self.ch.pub( "Client close control" )
        count = 0
        while count < START_TIME :
            rospy.sleep( 1.0 )
            count += 1
            print( "Count is " + str( count ) + " seconds" )
        self.ch.activate( True )
        self.ch.pub( "Client open control" )

        print( "set roll to 0" )
        self.ch.absolute_roll( 0 )
        print( "set pitch to 0" )
        self.ch.absolute_pitch( 0 )
    
        print( "set relative depth to -1.0" )
        self.ch.relative_depth( -1.0 )

        self.ch.pub( "Client command relative depth is -1.0 sleep for 2 second" )
        rospy.sleep( 2.0 )

        self.ch.pub( "Client command forward by force 4")
        self.ch.plane_xy( 4.0 )
        print( "Sleep for 11 second")
        count = 0
        while count < 10:
            rospy.sleep( 1.0 )
            count += 1
            print( "Count is " + str( count ) + " seconds" )
        self.ch.plane_xy( 0.0 )
        self.ch.pub( "Client command forward by 0" )
        
        self.ch.absolute_pitch( -1.57 )
        self.ch.pub( "Clinet command absolute pitch is -1.57" )
        print( "Sleep for 2 second" )
        count = 0
        rospy.sleep( 2.0 )
 
        self.ch.pub( "Client command forward by force -4")
        self.ch.plane_xy( -4.0 )
        print( "Sleep for 11 second")
        count = 0
        while count < 10:
            rospy.sleep( 1.0 )
            count += 1
            print( "Count is " + str( count ) + " seconds" )
        self.ch.plane_xy( 0.0 )
        self.ch.pub( "Client command forward by 0" )

        self.ch.activate( False )
            
if __name__ == "__main__":
    mission = Forward()
    mission.activate()
