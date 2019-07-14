#!/usr/bin/env python2
# FILE			: drop_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 13 (UTC+0)
# MAINTAINER	: K.Supasan

# README

# REFERENCE

import rospy
import math
from zeabus.math import general as zeabus_math  
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_drop import AnalysisDrop
from zeabus.vision.analysis_constant import *
from constant_speed import *

class Drop:

    def __init__( self ):

        self.vision = AnalysisDrop( "base_drop" )
        self.control = CommandInterfaces( "DROP")

        self.rate = rospy.Rate( 5 )
        self.status_mission = 0

    def start_mission( self ): # This use in case you only run single mission
        
        self.control.publish_data( "START doing mission drop" )

        self.control.activate( ['x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw'] )

        self.control.reset_state( 0 , 0 )

        target_depth = STRATEGY_DEPTH_FIND_DROP

        self.control.absolute_z( target_depth )
        self.control.publish_data( "START waiting depth")
        while( not self.control.check_z( 0.12 )  ):
            self.rate.sleep()

        self.control.publish_data( "START forward TO FIND DROP" )

        self.control.deactivate( ['x' , 'y' ] )

        found_picture = False
 
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_TARGET )
            self.vision.echo_data()

            if( self.vision.result['found'] ):
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False
                found_picture = True
                if( self.vision.result['center_x'] > 15 ):
                    force_y = -1.3
                elif( self.vision.result['center_x'] < -15 ):
                    force_y = 1.3
                else:
                    ok_y = True
                if( self.vision.result['center_y'] > 20 ):
                    force_x = 1
                elif( self.vision.result[ 'center_y' ] < -20 ):
                    force_x = -1
                else:
                    ok_x = True

                if( ok_x and ok_y ):
                    if( self.control.check_z( 0.12 ) ):
                        self.control.publish_data( "START xyz ok")
                        if( target_depth < DROP_START_DEPTH ):
                            break
                        else:
                            target_depth -= 0.3
                            self.control.absolute_z( target_depth )
                            self.control.publish_data("START command depth at " 
                                + str(target_depth) )
                    else:
                        self.control.publish_data( "START xy ok waiting depth")
    
                self.control.force_xy( force_x , force_y )
                self.control.publish_data( "START Command force ( x , y ) : " 
                    + repr( (force_x , force_y ) ) )
            else:
                found_picture = False
                self.control.force_xy( 1.5 , 0 )

        self.control.force_false()

        if( found_picture ):
            self.control.publish_data( "START Find target give process to operator")
            self.operator()
        else:
            self.control.publish_data( "Abort mission by command" )

        self.control.activate( [ 'x' , 'y' ] )
        self.control.absolute_z( -1 )
        self.control.relative_xy( 0 , 0 )
        self.control.sleep()

    def operator( self ):
        self.control.publish_data( "OPERATOR Welcome to operator mission")
    
        self.control.publish_data( "OPERATOR command z to " + str( DROP_START_DEPTH ) )
    
        self.control.absolute_z( DROP_START_DEPTH )
    
        # This function don't already to use
        if( DROP_HAVE_TO_ROTATION ):
            pass
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
                if( self.vision.result['center_x'] > 10 ):
                    force_y =  -1.5
                elif( self.vision.result['center_x'] < -10 ):
                    force_y = 1.5
                else:
                    ok_y = True
    
                if( self.vision.result['center_y'] > 10 ):
                    force_x = 1.2
                elif( self.vision.result['center_y'] < -10 ):
                    force_x = -1.2
                else:
                    ok_x = True
    
                if( ok_x and ok_y ):
                    self.control.publish_data( "OPERATOR Now object is center of frame")
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( self.vision.result['type'] ):
                            self.control.publish_data( "OPERATOR Yes I see 4 point")
                            self.control.deactivate( ['x' , 'y' , 'yaw' ] )
                            while( not rospy.is_shutdown() ):
                                self.rate.sleep()
                                self.vision.call_data( DROP_FIND_TARGET )
                                self.vision.echo_data()
                                if( self.vision.result['rotation'] > 0.1 ):
                                    self.control.publish_data( "OPERATOR rotation left")
                                    self.control.force_xy_yaw( 0 , 0 , 0.2 )
                                elif( self.vision.result['rotation'] < -0.1 ):
                                    self.control.publish_data( "OPERATOR rotation right")
                                    self.control.force_xy_yaw( 0 , 0 , -0.2 )
                                else:
                                    self.control.force_false()
                                    self.control.publish_data( "OPERATOR rotation is OK!!!")
                                    self.control.sleep()
                                    self.control.activate( ['x' , 'y' , 'yaw' ] )
                                    self.control.relative_yaw( 0 , False )
                                    self.control.sleep()
                                    break
                            self.control.publish_data( "OPERATOR will continue open ? drop?")
                            break
                        else:
                            self.control.publish_data( "OPERATOR we don't see 4 point")
                            self.control.relative_z( 0.2 )
                    else:
                        self.control.publish_data( "OPERATOR Waiting z are ok")
                else:
                    self.control.force_xy( force_x , force_y )
                    self.control.publish_data( "OPERATOR force command " 
                        + repr( ( force_x , force_y ) ) )
            else:
                count_unfound += 1
                self.control.publish_data( "OPERATOR Don't found picture")

        self.control.publish_data( "OPERATOR Choose Process")
        self.control.deactivate( ['x' , 'y' ] )
        if( DROP_WANT_OPEN ):
            self.control.publish_data( "OPERATOR chosee open" )
            self.open( 40 )
        else:
            self.control.publish_data( "OPERATOR chosee drop" )
            self.drop( -40 )

        self.control.absolute_z( DROP_START_DEPTH )

    def open( self , offset_center ):
        self.control.publish_data("DROP start mission")
        count_unfound = 0

        self.control.update_target()
        target_depth = self.control.target_pose[2] 

        while( ( not rospy.is_shutdown() ) and  count_unfound < 5 ):
            self.rate.sleep()
            if( target_depth > DROP_TARGET_DEPTH ):
                self.control.publish_data( "DROP call data to get target")
                self.vision.call_data( DROP_FIND_TARGET )
                max_x = offset_center + 10
                min_x = offset_center - 10
            else:
                self.control.publish_data( "DROP call data to get area drop")
                self.vision.call_data( DROP_FIND_OPEN )
                max_x = 10
                min_x = 10
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0 
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False

                if( self.vision.result['center_y'] > 10 ):
                    force_x = 0.8
                elif( self.vision.result['center_y'] < -10 ):
                    force_x = -0.8
                else:
                    ok_x = True
                if( self.vision.result[ 'center_x' ] > ( max_x ) ):
                    force_y = -1.2
                elif( self.vision.result[ 'center_x'] < ( min_x ) ):
                    force_y = 1.2
                else:
                    ok_y = True

                if( ok_x and ok_y ):
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth <= DROP_DEPTH_ACTION ):
                            self.control.publish_data( "DROP Now depth is target let to open")
                            break
                        else:
                            self.control.publish_data( "DROP new depth ( command ,  target) : " 
                                + repr( ( target_depth , DROP_TARGET_DEPTH ) ) )
                            target_depth -= 0.4  
                            self.control.absolute_z( target_depth )
                            self.control.sleep()
                    else:
                        self.control.publish_data( "DROP now center x y waiting depth" )
                else:
                    self.control.publish_data( "DROP command force " 
                        + repr( ( force_x , force_y ) ) )
                    self.control.force_xy( force_x , force_y ) 
            else:
                self.control.publish_data( "DROP center mission unfound " + str( count_unfound ))
                self.control.force_xy( 0 , 0 )
                count_unfound += 1

        success = False
        self.control.publish_data( "DROP let to open")
        count_ok = 0
        while( ( not rospy.is_shutdown() ) and count_ok < 5 and count_unfound < 5):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_DROP )
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0
                force_y = 0
                force_x = 0
                if( self.vision.result['center_y'] > 90 ):
                    force_x = 1.0
                elif( self.vision.result['center_y'] < 60 ):
                    force_x = -1.0
                else:
                    count_ok += 1
                if( self.vision.result['center_x'] < -20):
                    force_y = 1.3
                elif( self.vision.result['center_x'] > 20 ):
                    force_y = -1.3
                else:
                    pass
                self.control.publish_data( "DROP command force " 
                    + repr( ( force_x , force_y )  ) )
                self.control.force_xy( force_x , force_y )  
            else:
                self.control.force_xy( 0 , 0 )
                count_unfound += 1

            self.control.force_xy( 0 , 0 )
            self.control.publish_data("DROP don't found picture " + str( count_unfound ))
            self.control.force_false()
            self.control.publish_data( "!!!!!!!!!!! Now dropping !!!!!!!!!!!!!")
            rospy.sleep( 1 )

    def drop( self , offset_center ):
        self.control.publish_data("DROP start mission")
        count_unfound = 0

        target_depth = DROP_START_DEPTH 

        while( ( not rospy.is_shutdown() ) and  count_unfound < 5 ):
            self.rate.sleep()
            if( target_depth > DROP_TARGET_DEPTH ):
                self.control.publish_data( "DROP call data to get target")
                self.vision.call_data( DROP_FIND_TARGET )
                max_x = offset_center + 10
                min_x = offset_center - 10
            else:
                self.control.publish_data( "DROP call data to get area drop")
                self.vision.call_data( DROP_FIND_DROP )
                max_x = 10
                min_x = 10
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0 
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False

                if( self.vision.result['center_y'] > 10 ):
                    force_x = 0.8
                elif( self.vision.result['center_y'] < -10 ):
                    force_x = -0.8
                else:
                    ok_x = True
                if( self.vision.result[ 'center_x' ] > ( max_x ) ):
                    force_y = -1.2
                elif( self.vision.result[ 'center_x'] < ( min_x ) ):
                    force_y = 1.2
                else:
                    ok_y = True

                if( ok_x and ok_y ):
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth <= DROP_DEPTH_ACTION ):
                            self.control.publish_data( "DROP Now depth is target let to drop")
                            break
                        else:
                            self.control.publish_data( "DROP new depth ( command ,  target) : " 
                                + repr( ( target_depth , DROP_TARGET_DEPTH ) ) )
                            target_depth -= 0.4  
                            self.control.absolute_z( target_depth )
                            self.control.sleep()
                    else:
                        self.control.publish_data( "DROP now center x y waiting depth" )
                else:
                    self.control.publish_data( "DROP command force " 
                        + repr( ( force_x , force_y ) ) )
                    self.control.force_xy( force_x , force_y ) 
            else:
                self.control.publish_data( "DROP center mission unfound " + str( count_unfound ))
                self.control.force_xy( 0 , 0 )
                count_unfound += 1

        self.control.publish_data( "DROP let to drop")
        while( ( not rospy.is_shutdown() ) and count_unfound < 5 ):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_DROP )
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                force_y = 0
                if( self.vision.result['center_y'] > 80 ):
                    count_unfound += 1
                    self.control.publish_data( "DROP y over 80 : "+str( count_unfound ) )
                if( self.vision.result['center_x'] < -20):
                    force_y = 1.3
                elif( self.vision.result['center_x'] > 20 ):
                    force_y = -1.3
                else:
                    pass
                self.control.publish_data( "DROP command force " 
                    + repr( ( DROP_FORCE_BACKWARD , force_y )  ) )
                self.control.force_xy( DROP_FORCE_BACKWARD , force_y )  
            else:
                self.control.force_xy( 0 , 0 )
                count_unfound += 1

            self.control.force_xy( 0 , 0 )
            self.control.publish_data("DROP don't found picture " + str( count_unfound ))
            self.control.force_false()
            self.control.publish_data( "!!!!!!!!!!! Now dropping !!!!!!!!!!!!!")
            rospy.sleep( 1 )
             

if __name__=="__main__":
    rospy.init_node( "mission_drop" )
    mission = Drop( )
    mission.start_mission()
