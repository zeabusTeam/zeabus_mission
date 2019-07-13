#!/usr/bin/env python2
# FILE			: drop_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 13 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
from zeabus_math import general as zeabus_math  
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_drop import AnalysisDrop
from zeabus.vision.analysis_constant import *

class Drop:

    def __init__( self ):

        self.vision = AnalysisDrop( "base_drop" )
        self.control = CommandInterfaces( "DROP")

        self.rate = rospy.Rate( 5 )
        self.status_mission = 0

    def start_mission( self ): # This use in case you only run single mission
        
        self.control.publish_data( "START doing mission drop" )

        self.control.reset_state( 0 , 0 )

        target_depth = -1

        self.control.absolute_z( target_depth )
        self.control.publish_data( "START waiting depth")
        while( not rospy.is_shutdown() ):
            self.rate.sleep()

        self.control.publish_data( "START forward TO FIND DROP" )

        self.control.deactivate( ['x' , 'y' ] )

        found_picture = False
 
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_TARGET )
            self.vision.echo()

            if( self.vision.result['found'] ):
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False
                found_picture = True
                if( self.vision.result['center_x'] > 20 ):
                    force_y = -1.4
                elif( self.vision.result['center_x'] < -20 ):
                    force_y = 1.4
                else:
                    ok_y = True
                if( self.vision.result['center_y'] > 20 ):
                    force_x = 1.0
                elif( self.vision.result[ 'center_y' ] < -20 ):
                    force_x = -1.0
                else:
                    ok_x = True

                if( ok_x and ok_y ):
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth < -1.5 ):
                            break
                        else:
                            target_depth -= 0.3
                            self.control.absolute_z( target_depth )
                            self.control.publish_data("START command depth at " 
                                + str(target_depth) )
                self.command.force_xy( force_x , force_y )
                self.control.publish_data( "Command force ( x , y ) : " 
                    + repr( (force_x , fource_y ) ) )
            else:
                found_picture = False
                self.command.force_xy( 0.8 , 0 )

        self.command.force_false()

        if( found_picture ):
            self.control.publish_data( "Abort mission by command" )
        else:
            self.control.publish_data( "START Find target give process to operator")

        self.control.activate( [ 'x' , 'y' ] )

    def operator( self ):
            self.control.publish_data( "OPERATOR Welcome to operator mission")

            self.control.publish_data( "OPERATOR command z to " + str( DROP_START_DEPTH ) )

            self.control.absolute_z( DROP_START_DEPTH )

            # This function don't already to use
            if( DROP_HAVE_TO_ROTATION ):
                None
            finish = False
            count_unfound = 0
            while( ( not rospy.is_shutdown() ) and count_unfound < 3 ): 
                self.rate.sleep()
                self.vision.call_data( DROP_FIND_TARGET )
                self.vision.echo_data()
                if( self.vision.result['found'] ):
                    count_unfound = 0
                    force_x = 0
                    force_y = 0
                    ok_x = False
                    ok_y = False
                    if( self.vision.result['center_x'] > 20 ):
                        force_y =  -1.5
                    elif( self.vision.result['center_x'] < -20 ):
                        force_y = 1.5
                    else:
                        ok_y = True

                    if( self.vision.result['center_y'] > 20 ):
                        force_x = 1.2
                    elif( self.vision.result['center_y'] < -20 ):
                        force_x = -1.2
                    else:
                        ok_x = True
    
                    if( ok_x and ok_y ):
                        self.control.publish_data( "OPERATOR Now object is center of frame")
                        if( self.control.check_z( 0.12 ) ):
                            if( self.vision.result['type'] ):
                                self.control.publish_data( "OPERATOR Yes I see 4 point")
                                self.control.deactivate( ['x' , 'y' , 'yaw' ] )
                                while( not rospy.is_shutdown() ):
                                    self.rate.sleep()
                                    self.vision.call_data( DROP_FIND_TARGET )
                                    self.vision.echo_data()
                                    if( self.vision.result['rotation'] ):
                            else:
                                self.control.publish_data( "OPERATOR we don't see 4 point")
                                self.control.relative_z( 0.2 )
                else:
                    count_unfound += 1
                    self.control.publish_data( "OPERATOR Don't found picture")
                 
