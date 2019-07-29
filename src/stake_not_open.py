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

        found_picture = False

        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data( STAKE_FIND_TARGET )
            self.vision.echo_data()

            if( self.vision.result['found']):
                force_x = 0
                force_y = 0
                relative_z = 0
                ok_x = False
                ok_y = True
                ok_z = False
                found_pitcure = True
                if( self.vision.result['center'][0] > 10 ):
                    force_y = -1.0
                elif( self.vision.result['center'][0] < -10 ):
                    force_y = 1.0
                else:
                    ok_x = True

                if( not ok_x ):
                    pass
                elif( self.vision.result['area'] < STRATEGY_STAKE_AREA_FOCUS ):
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
                    force_y = TARGET_RIGHT
                elif( self.vision.result['center'][0] < -20 ):
                    force_y = TARGET_LEFT
                else:
                    ok_x = True

                if( not ok_x ):
                    pass
                elif( self.vision.result['area'] < STAKE_AREA_ROTATION ):
                    force_x = TARGET_FORWARD
                elif( self.vision.result['area'] > STAKE_AREA_ROTATION_OVER ):
                    force_x = TARGET_BACKWARD
                else:
                    ok_z = True

                if( self.vision.result['center'][1] > 50 ):
                    force_z = STAKE_Z_UP
                elif( self.vision.result['center'][1] < -80 ):
                    force_z = STAKE_Z_DOWN - 0.5
                elif( self.vision.result['center'][1] < -50 ):
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
                            if( abs( self.vision.result['center'][0] ) > 40 ):
                                ok_x = False
                            else:
                                ok_x = True
                            if( abs( self.vision.result['center'][0] ) > 80 ):
                                ok_y = False
                            else:
                                ok_y = True
                            self.control.publish_data( "OPERATOR status picture " 
                                + repr( (ok_x , ok_y ) ) )
                            if( ok_x and ok_y ):
                                force_yaw = 0
                                if( self.vision.result['rotation'] > 0.12 ):
                                    self.control.force_xyz_yaw( 0 , 0 , STAKE_Z_FORCE_0 , 0.15 )
                                    self.control.publish_data("OPERATOR rotation left")
                                elif( self.vision.result['rotation'] < -0.12 ):
                                    self.control.force_xyz_yaw( 0 , 0 , STAKE_Z_FORCE_0 , -0.15 )
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

    # Lock target is function to fire will received target and try to call if ever found 
    #   This function will return true
    def lock_target( self , target , area ):

        count_unfound = 0
        count_found = 0
        can_fire = False
        while ( not rospy.is_shutdown() ) and count_unfound < 2:
            self.rate.sleep()
            self.vision.call_data( target )
            self.vision.echo_data()

            if( self.vision.result['found'] ):
                count_found += 1
                count_unfound = 0

                if( count_found == 2 ):
                    can_fire = True

                force_x = 0 
                force_y = 0
                force_z = STAKE_Z_FORCE_0
                ok_x = False
                ok_y = False

                if self.vision.result['center'][0] < STAKE_TARGET_X[0] :
                    force_y = TARGET_LEFT
                elif self.vision.result['center'][0] > STAKE_TARGET_X[1]:
                    force_y = TARGET_RIGHT
                else:
                    ok_x = True

                if self.vision.result['center'][1] < STAKE_TARGET_Y[0]:
                    force_z = STAKE_Z_DOWN
                elif self.vision.result['center'][1] > STAKE_TARGET_Y[1]:
                    force_z = STAKE_Z_UP
                else:
                    ok_y = False

                if( ok_x and ok_y ):
                    if( self.vision.result['area'] >  area ):
                        can_fire = True
                        break
                    else:
                        force_x = TARGET_FORWARD 
                self.control.force_xyz( force_x , force_y , force_z )
                self.control.publish_data( "LOCK " + target 
                    + " on area " + str( self.vision.result['area'] ) 
                    + " command force " + repr( ( force_x , force_y , force_z ) ) )
            else:
                count_found = 0
                count_unfound += 1
                self.control.publish_data( "LOCK " + target + " unfound object!")
                self.control.force_xyz( 0 , 0 , STAKE_Z_FORCE_0 )

        if( can_fire ):
            self.control.publish_data( "LOCK " + target + " Fire torpido!!!!!!!!!!")
            self.control.command_torpido( True )
            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            while( not rospy.is_shutdown() ) and diff_time < 2 :
                self.rate.sleep()
                self.control.force_xyz( 0 , 0 , STAKE_Z_FORCE_0 )

        return can_fire

#    def heart( self ):
#
#        self.control.activate( ['x' , 'y' , 'z'] )
#        self.control.sleep()
#        self.control.publish_data( "HEART start mission go to depth " + str( STAKE_START_DEPTH ) )
#        
#        while not self.control.check_z( 0.12 ):
#            self.rate.sleep()
#
#        self.control.publish_data( "HEART Try find target again" )
#        self.control.deactivate( ['x' , 'y' ] ) 
#        start_time = rospy.get_rostime()
#        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
#        found_picture = False
#        while ( not rospy.is_shutdown() ) and diff_time < STAKE_LIMIT_TIME :
#            self.rate.sleep()
#            self.vision.call_data( STAKE_FIND_HEART )
#            self.vision.echo_data()
#            
#            if( self.vision.result['found'] ):
#                found_pitcure = True
#                if self.vision.result['center'][ 1 ] < -40 :
#                    self.control.relative_z( -0.3 )
#                elif( self.vision.result['center'][ 1 ] > 40 ):
#                    self.control.relative_z( 0.3 )
#                else:
#                    pass
#
#                force_x = 0
#                force_y = 0
#                if self.vision.result['center'][0] > 20 :
#                    force_y = TARGET_RIGHT
#                elif self.vision.result['center'][0] < -20 :
#                    force_y = TARGET_LEFT
#                else:
#                    pass
#
#                if self.vision.result['area'] > STAKE_AREA_ROTATION_OVER :
#                    force_x = TARGET_BACKWARD
#
#                self.control.force_xy( force_x , force_y )
#                break
        
    def oval( self ):
        self.control.deactivate( ['x' , 'y' , 'z'] ) 

        self.control.force_xyz(  0 , 0 , STAKE_Z_FORCE_0 )
        min_x = STAKE_OVAL_CENTER_X - 10
        max_x = STAKE_OVAL_CENTER_X + 10
        self.control.publish_data( "OVAL Mission start center x is " + str(STAKE_OVAL_CENTER_X))
        count_unfound = 0 
        while( not rospy.is_shutdown() ) and count_unfound < 3 :
            self.rate.sleep()
            self.vision.call_data( STAKE_FIND_TARGET )
            self.vision.echo_data()
            force_x = 0 
            force_y = 0
            force_z = STAKE_Z_FORCE_0
            ok_x = False
            ok_y = False
            ok_z = False
            if self.lock_target( STAKE_OVAL_DIRECTION , STAKE_OVAL_AREA ) :
                self.control.publish_data( "SPECIAL OVAL Have do mission end this process")
                break

            if not self.vision.result['found'] :
                count_unfound += 1
                self.control.publish_data( "Don't found picture count " + str( count_unfound ) )
                self.control.force_xyz( 0 , 0 , STAKE_Z_FORCE_0 )
                continue

            count_unfound = 0
            if( self.vision.result[ 'top' ] > 90 ):
                force_z = STAKE_Z_UP + 0.3
            elif( self.vision.result[ 'top' ] > 70 ):
                force_z = STAKE_Z_UP
            elif( self.vision.result[ 'top'] < 30 ):
                force_z = STAKE_Z_DOWN
            elif( self.vision.result['top'] < 0 ):
                force_z = STAKE_Z_DOWN - 0.3
            else:
                ok_y = True

            if( self.vision.result[ STAKE_OVAL_DIRECTION ] > max_x ):
                force_y = -0.8
            elif( self.vision.result[ STAKE_OVAL_DIRECTION ] < min_x ):
                force_y = 0.8
            else:
                ok_x = True

            if( ok_x and ok_y ):
                if( self.vision.result['area'] > 30 ):
                    force_x = -0.5
                elif( self.vision.result['area'] < 10 ):
                    force_x = 0.5
                else:
                    ok_z = True
            else:
                pass

            self.control.publish_data( "OVAL force xyz " + repr((force_x, force_y, force_z)))
            self.control.force_xyz( force_x , force_y , force_z )

        self.control.publish_data( "OVAL finish fire next I will backward")

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        while( ( not rospy.is_shutdown() ) and diff_time < STAKE_BACKWARD_TIME ):
            self.rate.sleep()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            self.control.force_xyz( -0.8 , 0 , STAKE_Z_DOWN )
            self.control.publish_data( "OVAL Backward time is " + str( diff_time ) )

if __name__=="__main__":
    rospy.init_node( "mission_drop" )
    mission = Stake( )
    mission.start_mission()
