#!/usr/bin/env python2
# FILE			: drop_only.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 14 (UTC+0)
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
        self.control = CommandInterfaces( "DROP" )

        self.rate = rospy.Rate( 5 )
        self.status_mission = 0

    def start_mission( self ):

        self.control.publish_data( "START doing mission drop" )

        self.control.activate( ['x' , 'y' , 'z' , 'roll' , 'pitch' , 'yaw'] )

        self.control.reset_state( 0 , 0 )
        self.control.sleep()

        target_depth = DROP_FIND_DEPTH

        self.control.absolute_z( target_depth )
        self.control.publish_data( "START waiting depth")
        while( not self.control.check_z( 0.12 )  ):
            self.rate.sleep()

        self.control.publish_data( "START forward TO FIND TARGET" )

        self.control.deactivate( ['x' , 'y'] )

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
                if( self.vision.result['center_x'] > 20 ):
                    force_y = SURVEY_RIGHT
                elif( self.vision.result['center_x'] < -20 ):
                    force_y = SURVEY_LEFT
                else:
                    ok_y = True
                if( self.vision.result['center_y'] > 20 ):
                    force_x = SURVEY_FORWARD
                elif( self.vision.result[ 'center_y' ] < -20 ):
                    force_x = SURVEY_BACKWARD
                else:
                    ok_x = True

                if( ok_x and ok_y ):
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth < DROP_START_DEPTH ):
                            self.control.publish_data( "START Now depth is ok next operator")
                            break
                        else:
                            target_depth += DROP_STEP_DEPTH
                            self.control.absolute_z( target_depth )
                            self.control.publish_data("START command depth (current : target)" 
                                + repr( ( target_depth , DROP_START_DEPTH ) ) )
                    else:
                        self.control.publish_data( "START xy ok waiting depth")
    
                self.control.force_xy( force_x , force_y )
                self.control.publish_data( "START Command force ( x , y ) : " 
                    + repr( (force_x , force_y ) ) )
            else:
                found_picture = False
                self.control.force_xy( 0 , 0 )

        self.control.force_false()

        if( found_picture ):
            self.control.publish_data( "START Find target give process to operator")
            self.operator()
        else:
            self.control.publish_data( "START Abort mission by command" )

        self.control.activate( [ 'x' , 'y' ] )
        self.control.relative_xy( 0 , 0 )
        self.control.absolute_z( DROP_FIND_DEPTH )
        self.control.sleep()

    def operator( self ):

        self.control.publish_data( "OPERATOR Welcome to operator mission")
    
        self.control.publish_data( "OPERATOR command z to " + str( DROP_START_DEPTH ) )
   
        self.control.deactivate( [ 'x' , 'y' ] )
 
        self.control.absolute_z( DROP_START_DEPTH )
    
        # This function don't already to use
        if DROP_HAVE_TO_ROTATION :
            self.control.relative_yaw( DROP_RADIAN_TO_ROTATION )
            self.control.sleep()
            self.control.publish_data( "OPERATOR relative yaw " + str( DROP_RADIAN_TO_ROTATION ) )
            while( not self.control.check_yaw( 0.12 ) ):
                self.rate.sleep()

        finish = False

        count_unfound = 0
        while( ( not rospy.is_shutdown() ) and count_unfound < 5 ): 
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
                    force_y = TARGET_RIGHT
                elif( self.vision.result['center_x'] < -10 ):
                    force_y = TARGET_LEFT
                else:
                    ok_y = True
    
                if( self.vision.result['center_y'] > 10 ):
                    force_x = TARGET_FORWARD
                elif( self.vision.result['center_y'] < -10 ):
                    force_x = TARGET_BACKWARD
                else:
                    ok_x = True
    
                if( ok_x and ok_y ):
                    self.control.publish_data( "OPERATOR Now object is center of frame")
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( self.vision.result['type'] ):
                            self.control.publish_data( "OPERATOR now process rotation")
                            self.control.relative_z( 0.05 , False )
                            self.control.deactivate( ['x' , 'y' , 'yaw' ] )
                            while( not rospy.is_shutdown() ):
                                self.rate.sleep()
                                self.vision.call_data( DROP_FIND_TARGET )
                                self.vision.echo_data()
                                if( self.vision.result['rotation'] > 0.15 ):
                                    self.control.publish_data( "OPERATOR rotation left")
                                    self.control.force_xy_yaw( 0 , 0 , DROP_FORCE_YAW )
                                elif( self.vision.result['rotation'] < -0.15 ):
                                    self.control.publish_data( "OPERATOR rotation right")
                                    self.control.force_xy_yaw( 0 , 0 , DROP_FORCE_YAW * -1.0 )
                                else:
                                    self.control.force_false()
                                    self.control.publish_data( "OPERATOR rotation is OK!!!")
                                    self.control.activate( ['x' , 'y' , 'yaw' ] )
                                    self.control.sleep()
                                    break
                            self.control.publish_data( "OPERATOR will continue open ? drop?")
                            break
                        else:
                            self.control.publish_data( "OPERATOR we don't see 4 point")
                            if( self.control.check_z( 0.12 ) ):
                                self.control.publish_data( "OPERATOR add depth")
                                self.control.relative_z( DROP_STEP_DEPTH * -1.0 )
                            else:
                                self.control.publish_data( "OPERATOR Waiting depth" )
                    else:
                        self.control.publish_data( "OPERATOR Waiting z are ok")
                else:
                    self.control.force_xy( force_x , force_y )
                    self.control.publish_data( "OPERATOR force command : " 
                        + repr( ( force_x , force_y ) ) )
            else:
                count_unfound += 1
                self.control.force_xy( 0 , 0 )
                self.control.publish_data( "OPERATOR Don't found picture")

        self.control.publish_data( "OPERATOR Choose Process")
        self.control.deactivate( ['x' , 'y' ] )
        if( count_unfound == 5 ):
            self.control.publish_data( "OPERATOR stop process because don't found picture" )
        else:
            self.control.publish_data( "OPERATOR chosee drop" )
            self.control.update_target()
            self.drop( DROP_CENTER_X_DROP , self.control.target_pose[2] )
            self.open( DROP_CENTER_X_OPEN )

        self.control.absolute_z( DROP_START_DEPTH )

    def drop( self , offset_center , start_depth ):
        self.control.publish_data("DROP start mission start at depth " + str( start_depth ) )
        count_unfound = 0

        target_depth = start_depth
        self.control.deactivate( [ 'x' , 'y'] )
        while( ( not rospy.is_shutdown() ) and  count_unfound < 5 ):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_TARGET )
            max_x = offset_center + 5
            min_x = offset_center - 5
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0 
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False

                if self.vision.result['center_y'] > 10 :
                    force_x = TARGET_FORWARD
                elif self.vision.result['center_y'] < -10 :
                    force_x = TARGET_BACKWARD
                else:
                    ok_x = True
                if self.vision.result[ 'center_x' ] > max_x :
                    force_y = TARGET_RIGHT
                elif self.vision.result[ 'center_x'] < min_x :
                    force_y = TARGET_LEFT
                else:
                    ok_y = True

                if( ok_x and ok_y ):
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth <= DROP_ONLY_DEPTH ):
                            self.control.publish_data( "DROP Now depth is target let to drop")
                            count_unfound = 5
                            break
                        else:
                            self.control.publish_data( 
                                "OPEN new depth ( command, first_target , second_target ) : " 
                                + repr( ( target_depth 
                                    , DROP_TARGET_DEPTH , DROP_ONLY_DEPTH ) ) )
                            target_depth += DROP_STEP_DEPTH 
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

        if( count_unfound == 5 ):
            self.control.absolute_z( DROP_ONLY_DEPTH )
            self.control.publish_data( "DROP waiting for z before backward")

            while( not self.control.check_z( 0.12 ) ):
                self.rate.sleep()

            self.control.publish_data( "DROP Process to backward before drop")
            start_time = rospy.get_rostime()
            diff_time = (rospy.get_rostime() - start_time).to_sec()
            while( not rospy.is_shutdown() ) and ( diff_time < DROP_BACKWARD_TIME ):
                self.rate.sleep()
                self.control.force_xy( DROP_BACKWARD_FORCE , 0 )
                diff_time = (rospy.get_rostime() - start_time).to_sec()
                self.control.publish_data( "DROP Process backward time is " + str( diff_time ) )

            self.control.force_xy( 0 , 0 ) 
            self.control.publish_data( "DROPING !!!!!!!!!!!!!!!!!!!!!!!!!!! Finish")
            self.control.sleep()

        self.control.absolute_z( DROP_TARGET_DEPTH )

    def open( self , offset_center ):
        self.control.publish_data("OPEN start mission start at depth "
            + str( DROP_TARGET_DEPTH ) )
        count_unfound = 0

        target_depth = DROP_START_DEPTH
        self.control.absolute_z( target_depth )
        self.control.publish_data( "OPEN waiting depth for doing again")
        self.control.sleep()
        while( not self.control.check_z( 0.12 ) ):
            self.rate.sleep()

        self.control.deactivate( [ 'x' , 'y'] )
        # This process is process little find target
        count_unfound = 0
        self.control.publish_data( "OPEN little find target")
        while not rospy.is_shutdown() and count_unfound < 50 :
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_TARGET )
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0
                force_x = 0 
                force_y = 0
                ok_x = False
                ok_y = False
                if self.vision.result['center_x'] < -15:
                    force_y = TARGET_LEFT
                elif self.vision.result['center_x'] > 15 :
                    force_y = TARGET_RIGHT
                else:
                    ok_y = True

                if self.vision.result['center_y'] < -15 :
                    force_x = TARGET_BACKWARD
                elif self.vision.result['center_y'] > 15 :
                    force_x = TARGET_FORWARD
                else:
                    ok_x = True

                if( ok_x and ok_y ):
                    self.control.publish_data( "OPEN now center of target")
                    break
                else:
                    self.control.publish_data( "OPEN command force " + repr((force_x , force_y)))
                    self.control.force_xy( force_x , force_y )
            else:
                count_unfound+=1
                self.control.publish_data( "OPEN Don't found target " + str( count_unfound ) )
                self.control.force_xy( TARGET_FORWARD , DROP_FORCE_FOR_FIND )

        while( ( not rospy.is_shutdown() ) and  count_unfound < 5 ):
            self.rate.sleep()
            self.vision.call_data( DROP_FIND_TARGET )
            self.vision.echo_data()
            max_x = offset_center + 10
            min_x = offset_center - 10
            self.vision.echo_data()
            if( self.vision.result['found'] ):
                count_unfound = 0 
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False

                if( self.vision.result['center_y'] > DROP_CENTER_Y + 10 ):
                    force_x = 0.4
                elif( self.vision.result['center_y'] < DROP_CENTER_Y - 10 ):
                    force_x = -0.4
                else:
                    ok_x = True
                if( self.vision.result[ 'center_x' ] > ( max_x ) ):
                    force_y = -0.8
                elif( self.vision.result[ 'center_x'] < ( min_x ) ):
                    force_y = 0.8
                else:
                    ok_y = True

                if( ok_x and ok_y ):
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth <= DROP_ACTION_DEPTH ):
                            self.control.publish_data( "OPEN Now depth is target let to open")
                            count_unfound = 5
                            break
                        else:
                            self.control.publish_data( 
                                "OPEN new depth ( command, first_target , second_target ) : " 
                                + repr( ( target_depth 
                                    , DROP_TARGET_DEPTH , DROP_ACTION_DEPTH ) ) )
                            target_depth -= 0.4  
                            self.control.absolute_z( target_depth )
                            self.control.sleep()
                    else:
                        self.control.publish_data( "OPEN now center x y waiting depth" )
                else:
                    self.control.publish_data( "OEPN command force " 
                        + repr( ( force_x , force_y ) ) ) 
                    self.control.force_xy( force_x , force_y ) 
            else:
                self.control.publish_data( "OPEN center mission unfound " +str(count_unfound) )
                self.control.force_xy( 0 , 0 )
                count_unfound += 1

        if( count_unfound == 5 ):
            self.control.publish_data( "OPEN now picture is unfound ? try to open" )
            self.control.absolute_z( DROP_ACTION_DEPTH )
            self.control.force_xy( 0 , 0 )
            self.control.sleep()

            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()

            while( ( not rospy.is_shutdown() ) and diff_time < DROP_TIME_OUT ):
                self.rate.sleep()
                self.control.force_xy( -0.2 , -1.0*DROP_FORCE_OPEN )
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.control.publish_data( "OPEN move left time is " + str( diff_time ))
            self.control.force_xy( 0 , 0 )

            self.control.publish_data( "OPEN Waiting z")
            while( not self.control.check_z( 0.12 ) ):
                self.rate.sleep()

            self.control.absolute_z( DROP_ACTION_DEPTH )

            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            while( ( not rospy.is_shutdown() ) and diff_time < DROP_TIME_OPEN ):
                self.rate.sleep()
                if( diff_time < DROP_TIME_OPEN / 2 ):
                    self.control.force_xy( -0.05 , DROP_FORCE_OPEN )
                else:
                    self.control.absolute_z( DROP_ACTION_DEPTH + 0.25 )
                    self.control.force_xy( -0.1 , DROP_FORCE_OPEN * 1.5)
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.control.publish_data( "OPEN move right time is " + str( diff_time ))

if __name__=="__main__":
    rospy.init_node( "mission_drop" )
    mission = Drop( )
    mission.start_mission()
