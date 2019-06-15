#!/usr/bin/env python2
# FILE			: square_2.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 15 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

from zeabus.control.command_interfaces import CommandInterfaces

import rospy

import math

if __name__=='__main__':

    rospy.init_node( "square_2")

    name_node = rospy.get_name()

    auv_control = CommandInterfaces( name_node )

    auv_control.reset_state( 0 , 0 )

    auv_control.echo_data()

    print( "Waiting z" )

    auv_control.absolute_z( -2 )
    while( not rospy.is_shutdown() ):
        if( auv_control.check_z( 0.15 ) ):
            break
        else:
            rospy.sleep( 0.5 )

    print( "Waiting xy move right" )
    auv_control.relative_xy( 0 , -1.5 )
    auv_control.echo_data()
    while( not rospy.is_shutdown() ):
        if( auv_control.check_xy( 0.1 , 0.1 ) ):
            break
        else:
            rospy.sleep( 0.5 )

    print( "Waiting xy move forward" )
    auv_control.relative_xy( 3 , 0 )
    auv_control.echo_data()
    while( not rospy.is_shutdown() ):
        if( auv_control.check_xy( 0.1 , 0.1 ) ):
            break
        else:
            rospy.sleep( 0.5 )

    print( "Waiting xy move left" )
    auv_control.relative_xy( 0 , 3 )
    auv_control.echo_data()
    while( not rospy.is_shutdown() ):
        if( auv_control.check_xy( 0.1 , 0.1 ) ):
            break
        else:
            rospy.sleep( 0.5 )

    print( "Waiting xy move backward" )
    auv_control.relative_xy( -3 , 0 )
    auv_control.echo_data()
    while( not rospy.is_shutdown() ):
        if( auv_control.check_xy( 0.1 , 0.1 ) ):
            break
        else:
            rospy.sleep( 0.5 )

    print( "Waiting xy move left" )
    auv_control.relative_xy( 0 , -1.5 )
    auv_control.echo_data()
    while( not rospy.is_shutdown() ):
        if( auv_control.check_xy( 0.1 , 0.1 ) ):
            break
        else:
            rospy.sleep( 0.5 )

