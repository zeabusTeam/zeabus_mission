#!/usr/bin/env python2
# FILE			: exposed_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 26 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   2019 06 26 Code path start mission

# REFERENCE

from __future__ import print_function

import math
import rospy
from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_coffin import AnalysisCoffin
from constant_speed import *

class Exposed:

    def __init__( self ):

        self.vision = AnalysisCoffin( "base_coffin")
        self.control = CommandInterfaces( "PATH" )

        self.rate = rospy.Rate( 5 )

    def start_mission( self):

        self.control.publish_data( "START welcome to start mission")

        self.control.publish_data( "START command detph to " + str( EXPOSED_START_DEPTH ) )
        self.control.absolute_z( EXPOSED_START_DEPTH )
        self.control.sleep()

        while( not self.control.check_z( 0.12 ) ):
            self.rate.sleep()

        self.control.deactivate( [ 'x' , 'y' ] )

        self.control.publish_data( "START forwrd to find exposed")
        count_found = 0
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 
        while( not rospy.is_shutdown() and diff_time < STRATEGY_TIME_FORWARD and count_found<5 ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.result['num_object'] > 0 ):
                count_found += 1
                force_x = 0
                force_y = 0
                ok_x = False
                ok_y = False
                self.control.publish_data( "START found target command force "  
                    + repr( ( force_x , force_y ) ) )
                if( self.vision.result['center_x'] < -20 ):
                    force_y = SURVEY_LEFT
                elif self.vision.result['center_x']  > 20 :
                    force_y = SURVEY_RIGHT
                else:
                    ok_y = True
            
                if( self.vision.result['center_y'] < -20 ):
                    force_x = SURVEY_BACKWARD
                elif self.vision.result['center_y'] > 20:
                    force_x = SURVEY_FORWARD
                else:
                    ok_x = True 

                if( ok_x and ok_y ):
                    count_found = 5
                    self.control.publish_data( "START now center point of coffin")
                else:
                    self.control.force_xy( force_x , force_y )
                    self.control.publish_data( "START Have picture command force " + repr( (
                        force_x , force_y ) ) )
            else:
                count_found = 0
                self.control.force_xy( SURVEY_FORWARD , 0 )
                self.control.publish_data( "START Don't found picture command survey forward" )
            diff_time = (rospy.get_rostime() - start_time).to_sec()

        self.control.activate( [ 'x' , 'y' ] )
        
        if( count_found == 5 ):
            self.control.publish_data( "START finish and found object let operator")
            self.operator()
        else:
            self.control.publish_data( "START finish and don't found object let find_coffin")
            self.find()

    def find( self ): 
        self.control.publish_data( "FIND start find expose")

        mode = 0 # 0 -> survey left 1 -> survey forward 2 -> survey right

        self.control.deactivate( ['x' , 'y' ] )

        count_round = 0
        count_found = 0
        start_time = rospy.get_rostime()        
        while( not rospy.is_shutdown() and count_round < EXPOSED_LIMIT_ROUND ):
            self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result['num_object'] > 0 ):
                count_found += 1
                force_x = 0 
                force_y = 0
                if( self.vision.result['center_x'] > 10 ):
                    force_y = SURVEY_RIGHT
                elif self.vision.result['center_x' ] < -10 :
                    force_y = SUPER_LEFT
                else:
                    pass

                if( self.vision.result['center_y'] > 10 ):
                    force_x = SURVEY_FORWARD
                elif self.vision.result['center_y' ] < -10 :
                    force_x = SURVEY_BACKWARD
                else:
                    pass
                self.control.publish_data( "FIND force command " + repr(( force_x , force_y )))
                continue
            else:
                count_found = 0

            if( count_found == 3):
                self.control.publish_data( "FIND Break object")
                self.control.force_xy( 0 , 0 ) 
                break
            
            if( mode == 0 ):
                self.control.force_xy( 0, SURVEY_LEFT )
            elif( mode == 1 ):
                self.control.force_xy( SURVEY_FORWARD , 0 )
            else:
                self.control.force_xy( 0 , SURVEY_RIGHT ) 

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            if( diff_time > EXPOSED_LIMIT_TIME ):
                mode = ( mode + 1 ) % 3
                self.control.publish_data( "FIND Change mode to " + str( mode ) )
            else:
                self.control.publish_data( "FIND Diff time is " + str( diff_time ) )

        result = False
        if( count_found == 3 ):
            self.operator()
            result = True

        self.control.activate( ['x' , 'y'] )

        return result

    def operator( self ):

        self.control.publish_data( "OPERATOR Start mission")

        self.control.deactivate( ['x' , 'y'] )

        count_unfound = 0        

        self.control.update_target()

        target_depth = self.control.target_pose[ 2 ]

        while not rospy.is_shutdown()  and count_unfound < 3 :
            self.vision.call_data()
            self.vision.echo_data()

            if( self.vision.result['num_object'] == 1 ):
                ok_x = False
                ok_y = False
                force_x = 0
                force_y = 0

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
                    self.control.force_xy( 0 , 0 )
                    if( self.control.check_z( 0.12 ) ):
                        if( target_depth >  EXPOSED_TARGET_DEPTH ):
                            target_depth -= 0.2
                            self.control.absolute_z( target_depth )
                            self.control.publish_data("OPERATOR command z " + str(target_depth) )
                        else:
                            self.control.publish_data( "OPERATOR now process rotation")
                            self.control.deactivate( ['x' , 'y' , 'yaw' ] )
                            success_rotation = False
                            while( not rospy.is_shutdown() ):
                                self.rate.sleep()
                                self.vision.call_data()
                                self.vision.echo_data()
                                if( self.vision.result['object_1']['rotation'] > 0.2 ):
                                    self.control.publish_data("OPERATOR rotation left")
                                    self.control.force_xy_yaw( 0 , 0 , EXPOSED_FORCE_YAW )
                                elif( self.vision.result['object_1']['rotation'] < -0.2 ):
                                    self.control.publish_data( "OPERATOR rotation right" )
                                    self.control.force_xy_yaw( 0 , 0 ,-EXPOSED_FORCE_YAW )
                                else:
                                    success_rotation = True
                                    break
                            self.control.activate( ['x' , 'y', 'yaw'] )
                            self.control.sleep()
                            self.control.deactivate( ['x', 'y' ] )
                            if( success_rotation ):
                                self.tune_center()                   
                                break 
                    else: 
                        self.control.publish_data("OPERATOR now center waiting depth")
                    
                else:
                    self.control.publish_data( "OPERATOR found single object force " + repr( (
                        force_x , force_y ) ) )
                    self.control.force_xy( force_x , force_y )
             
            elif self.vision.result['num_object'] == 2 :
                self.control.publish_data( "OPERATOR found two object" )
                break
            else:
                count_unfound += 1

        self.control.activate( ( 'x' , 'y' ) )

    def tune_center( self ):
        self.control.publish_data("TUNE_CENTER welcome to tune_center function")

        self.control.absolute_z( EXPOSED_TARGET_DEPTH )

        self.control.publish_data("TUNE_CENTER waiting depth" )
        while( not self.control.check_z( 0.12 ) ):
            self.rate.sleep()

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec()
        can_go_up = False
        while( ( not rospy.is_shutdown() ) and diff_time < EXPOSED_LIMIT_TIME_TO_FIND ):
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.result['num_object'] == 2 ):
                can_go_up = True
                self.control.force_xy( 0 , 0 )
                self.control.publish_data( "TUNE_CENTER found two object")
                break
            elif( self.vision.result['num_object'] == 1 ):
                if( ( self.vision.result['object_1']['center_x'] * EXPOSED_CENTER_X_DIRECTION ) >
                    EXPOSED_CENTER_X_NEW_VALUE ) :
                    can_go_up = True 
                    self.control.force_xy( 0 , 0 )

                    start_time = rospy.get_rostime()
                    diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                    while( not rospy.is_shutdown() ) and diff_time < 5 :
                        self.rate.sleep()
                        self.control.force_xy( 0 , EXPOSED_FORCE_TO_BACK )
                        diff_time = ( rospy.get_rostime() - start_time ).to_sec()

                    self.control.publish_data( "TUNE_CENTER found new object")
                    break
                else:
                    self.control.publish_data( "TUNE_CENTER found same coffin")
            else:
                diff_time = ( rospy.get_rostime() - start_time ).to_sec()
                self.control.publish_data( "TUNE_CENTER Don'found object " + str( diff_time ) )
                self.control.force_xy( 0 , EXPOSED_FORCE_TO_FIND )
        self.control.force_xy( 0 , 0 )
        if( not can_go_up ):
            self.control.publish_data( "TUNE_CENTER We want to find again")
            while not self.control.check_yaw( 0.12 ):
                self.rate.sleep()
            start_time = rospy.get_rostime()
            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            while( not rospy.is_shutdown() ) and ( diff_time < EXPOSED_LIMIT_TIME_TO_FIND * 2.5 ):
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.result['num_object'] == 1 ):
                    while not rospy.is_shutdown() :
                        self.vision.call_data()
                        self.vision.echo_data()
                        self.control.publish_data( "TUNE_CENTER have to tune center again")
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
                            self.control.publish_data("TUNE_CENTER now ok")
                            break
                        else:
                            self.control.publish_data( "TUNE_CENTER force " 
                                + repr( (force_x, force_y ) ) )
                            self.control.force_xy( force_x , force_y )
                else:
                    self.control.force_xy( 0 , EXPOSED_FORCE_TO_BACK )

                diff_time = ( rospy.get_rostime() - start_time ).to_sec()

            self.control.publish_data( "TUNE_CENTER have to move out and go up")
            while( not rospy.is_shutdown() ):
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.result['num_object'] == 0 ):
                    self.control.publish_data( "TUNE_CENTER don't found object" )
                    break
                else:
                    self.control.publish_data( "TUNE_CENTER still found object" )

        self.control.absolute_z( -0.2 )
        self.control.publish_data( "TUNE_CENTER command to depth")
        while( not self.control.check_z( 0.2 ) ):
            self.rate.sleep()

        self.control.publish_data( "TUNE_CENTER have to sleep")
        self.control.deactivate( ['x' , 'y' , 'z' , 'yaw'] )
        rospy.sleep( 6 )
        self.control.activate( ['x' , 'y' , 'z' , 'yaw' ] )
        self.control.publish_data( "TUNE_CENTER have to wakeup")
        self.control.absolute_z( -0.5 )
        
if __name__=="__main__" :
    rospy.init_node( "mission_exposed" )

    mission = Exposed( )
    mission.start_mission()
