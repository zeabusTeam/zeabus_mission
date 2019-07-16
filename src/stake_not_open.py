#!/usr/bin/env python2
# FILE			: stake_not_open.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 16 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math

from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_stake import AnalysisStake
from zeabus.vision.analysis_constant import *
from constant_speed import *

class Stake:

    def __init__( self ):

        self.vision = AnalysisStake( "base_stake" )
        self.control = CommandInterfaces( "STAKE" )

        self.rate = rospy.Rate( 5 )
        self.status_mission = 0

    def start_mission( self ):
        self.control.publish_data( "START doing mission stake" )

        self.control.activate( ['x' , 'y', 'z', 'roll' , 'pitch' , 'yaw' ] )

        self.control.absolute_z( STRATEGY_STAKE_DEPTH_FIND )

        self.control.sleep()

        self.control.publish_data( "START Waiting depth")
        while( not self.control.check_z( 0.12 ) ):
            self.rate.sleep()

        self.control.publish_data( "START forward to Find target" )

        self.control.deactivate( [ 'x' , 'y' ] )

        found_pitcure = False

        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data( STAKE_FIND_TARGET )
            self.vision.echo_data()

            if( self.vision.result['found']):
                force_x = 0
                force_y = 0
                relative_z = 0
                ok_x = False
                ok_y = False
                ok_z = False
                found_pitcure = True
                if( self.vision.result['center'][0] > 10 ):
                    force_y = -1.5
                elif( self.vision.result['center'][0] < -10 ):
                    force_y = 1.5
                else:
                    ok_x = True

                if( self.vision.result['center'][1] > 15 ):
                    relative_z = 0.2
                elif( self.vision.result['center'][1] < 15 ):
                    relative_z = -0.2
                else:
                    ok_y = True

                if( self.vision.result['area'] < STRATEGY_STAKE_AREA_FOCUS ):
                    force_x = 1
                    self.control.publish_data( "STRATEGY area now {:6.3f} < {:6.3f}".format(
                        self.vision.result['area'] , STRATEGY_STAKE_AREA_FOCUS ) )
                else:
                    ok_z = True

                if( ok_x and ok_z ):
                    self.control.force_xy( 0 , 0 )
                    self.control.relative_z( 0 , False )
                    self.control.publish_data( "Start ok all axis next will go to for operator")
                    break 

                self.control.force_xy( force_x , force_y )
                self.control.publish_data( "START command force x y : " 
                    + repr( ( force_x , force_y ) ) )   

#                if( ok_y ):
#                    self.control.publish_data( "START center y is center now")
#                else:
#                    if( self.control.check_z( 0.1 ) ):
#                        self.control.publish_data( "START relative z  is " + str( relative_z ) )
#                        self.control.relative_z( relative_z )
#                    else:
#                        self.control.publish_data( "START Waiting z")
#
            else:
                self.control.force_xy( STRATEGY_FORCE_STAKE , 0 )
                self.control.publish_data( "START command force is " 
                    + str( STRATEGY_FORCE_STAKE ) )

        self.control.activate( [ 'x' , 'y' ] )
        if( found_pitcure ):
            self.operator()

    def operator( self ):
        self.control.publish_data( "OPERATOR Welcome to operator of STAKE mission" )

        self.control.deactivate( ['x' , 'y' , 'z' ] )

        self.control.publish_data( "OPERATOR This step use to rotation")
        count_unfound = 0
        self.control.force_xyz( 0 , 0 , STAKE_Z_FORCE_0 )
        while( ( not rospy.is_shutdown() ) and count_unfound < 5 ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result['found'] ):
                count_unfound = 0
                force_x = 0
                force_y = 0
                force_z = STAKE_Z_FORCE_0
                ok_y = False
                ok_z = False
                ok_x = False
                if( self.vision.result['center'][0] > 20 ):
                    force_y = -1
                elif( self.vision.result['center'][0] < -20 ):
                    force_y = 1
                else:
                    ok_x = True
                
                if( self.vision.result['area'] < STAKE_AREA_ROTATION ):
                    force_x = 0.8
                elif( self.vision.result['area'] > STAKE_AREA_ROTATION_OVER ):
                    force_x = -0.8
                else:
                    ok_z = True

                if( self.vision.result['center'][1] > 20 ):
                    force_z = STAKE_Z_UP
                elif( self.vision.result['center'][1] < -60 ):
                    force_z = STAKE_Z_DOWN - 0.5
                elif( self.vision.result['center'][1] < -20 ):
                    force_z = STAKE_Z_DOWN
                else:
                    ok_y = True

                if( ok_x and ok_y and ok_z ):
                    self.control.publish_data( "OPERATOR ok center next I will go to rotation")
                    success_rotation = False
                    self.control.deactivate( ['x' , 'y' , 'z' , 'yaw' ] )
                    self.control.force_xyz( 0 , 0 , STAKE_Z_FORCE_0 )
                    while( not rospy.is_shutdown() and count_unfound < 3 ):
                        self.rate.sleep()
                        self.vision.call_data( STAKE_FIND_TARGET )
                        self.vision.echo_data()
                        if( self.vision.result['found'] ):
                            count_unfound = 0
                            if( abs( self.vision.result['center'][0] ) > 50 ):
                                ok_x = False
                            else:
                                ok_x = True
                            if( abs( self.vision.result['center'][0] ) > 50 ):
                                ok_y = False
                            else:
                                ok_y = True
                            self.control.publish_data( "OPERATOR status picture " 
                                + repr( (ok_x , ok_y ) ) )
                            if( ok_x and ok_y ):
                                force_yaw = 0
                                if( self.vision.result['rotation'] > 0.25 ):
                                    self.control.force_xyz_yaw( 0 , -1 , STAKE_Z_FORCE_0 , 0.25 )
                                    self.control.publish_data("OPERATOR rotation left")
                                elif( self.vision.result['rotation'] < -0.25 ):
                                    self.control.force_xyz_yaw( 0 , 1 , STAKE_Z_FORCE_0 , -0.25 )
                                    self.control.publish_data("OPERATOR rotation right")
                                else:
                                    self.control.publish_data("OPERATOR success rotation")
                                    success_rotation = True   
                                    break 
                            else:
                                success_rotation = False
                                break
                        else:
                            count_unfound += 1
                            self.control.force_xyz_yaw( 0, 0, STAKE_Z_FORCE_0 , 0 )
                    self.control.activate( ['x' , 'y' , 'z' , 'yaw' ] )
                    self.control.sleep()
                    self.control.deactivate( ['x' , 'y' , 'z' ] )
                    if( success_rotation ):
                        self.control.publish_data("OPERATOR !!!!!!!!!!!!!!!!!!! Breaking")
                        break
                    else:
                        self.control.publish_data( "Failure rotation tune center again")  
                else:
                    self.control.publish_data( 
                        "OPERATOR command xyz : {:6.3f} {:6.3f} {:6.3f}".format( 
                            force_x , force_y , force_z ) ) 
                    self.control.force_xyz( force_x , force_y , force_z )                    
            else:
                count_unfound += 1

        if( count_unfound == 5 ):
            self.control.publish_data( "Operator Failure to do this mission" )
        else:
            self.oval()

    def heart( self ):
        pass

    def oval( self ):
        self.control.deactivate( ['x' , 'y' , 'z'] ) 

        self.control.force_xyz(  0 , 0 , STAKE_Z_FORCE_0 )
        min_x = STAKE_OVAL_CENTER_X - 10
        max_x = STAKE_OVAL_CENTER_X + 10
        self.control.publish_data( "OVAL Mission start center x is " + str(STAKE_OVAL_CENTER_X))
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data( STAKE_FIND_TARGET )
            self.vision.echo_data()
            force_x = 0 
            force_y = 0
            force_z = STAKE_Z_FORCE_0
            ok_x = False
            ok_y = False
            ok_z = False
            if( self.vision.result[ 'top' ] > 90 ):
                force_z = STAKE_Z_UP + 0.3
            elif( self.vision.result[ 'top' ] > 65 ):
                force_z = STAKE_Z_UP
            elif( self.vision.result[ 'top'] < 35 ):
                force_z = STAKE_Z_DOWN
            elif( self.vision.result['top'] < 0 ):
                force_z = STAKE_Z_DOWN - 0.3
            else:
                ok_y = True

            if( self.vision.result[ STAKE_OVAL_DIRECTION ] > max_x ):
                force_y = -1.3
            elif( self.vision.result[ STAKE_OVAL_DIRECTION ] < min_x ):
                force_y = 1.3
            else:
                ok_x = True

            if( ok_x and ok_y ):
                self.control.publish_data( "OVAL Now x and y is center")
                self.vision.call_data( STAKE_OVAL_DIRECTION )
                if( self.vision.result[ 'found' ] ):
                    while( not rospy.is_shutdown() ):
                        self.rate.sleep()
                        self.vision.call_data( STAKE_OVAL_DIRECTION )
                        if( self.vision.result[ 'found' ] ):
                            ok_x = False
                            ok_y = False
                            ok_z = False
                            force_x = 0
                            force_y = 0
                            force_z = STAKE_Z_FORCE_0
                            if( self.vision.result['center'][0] < (STAKE_TARGET_POINT[0]-10)):
                                force_y = 0.8
                            elif( self.vision.result['center'][0] > (STAKE_TARGET_POINT[1]+10)):
                                force_y = -0.8
                            else:
                                ok_y = True
                
                            if( self.vision.result['center'][1] > (STAKE_TARGET_POINT[1]-10)):
                                force_z = STAKE_Z_UP
                            elif( self.vision.result['center'][1] < (STAKE_TARGET_POINT[1]+10)):
                                force_z = STAKE_Z_DOWN
                            else:
                                ok_z = True

                            if( self.vision.result['area'] > ( STAKE_OVAL_AREA ) ):
                                force_x = 0.5
                            else:
                                ok_x = True

                        if( ok_x and ok_y and ok_z ):
                            self.control.force_xyz( force_x , force_y ,force_z )
                            self.control.publish_data( "OVAL !!!!!!!!!!!!!! Command fireeeeee")
                            rospy.sleep( 2 )
                        else:
                            self.control.publish_data( 
                                "SPECIAL OVAL force {:6.3f} {:6.3f} {:6.3f}".format(
                                    force_x , force_y , force_z ) )
                            
                else:
                    self.control.publish_data( "OVAL don't found target found")
                    self.control.force_xyz( 1.0 , force_y , force_z )
            else:
                self.control.publish_data( "OVAL force xyz " + repr((force_x, force_y, force_z)))
                self.control.force_xyz( 0.2 , force_y , force_z )
                
                
if __name__=="__main__":
    rospy.init_node( "mission_drop" )
    mission = Stake( )
    mission.start_mission()
