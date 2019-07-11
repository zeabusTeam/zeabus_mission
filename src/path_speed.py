#!/usr/bin/env python2
# FILE			: path_speed.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, June 26 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   2019 06 26 Code path start mission

# REFERENCE

import rospy
import math
from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_path import AnalysisPath
from constant_speed import *

class Path:

    def __init__( self ):

        self.vision = AnalysisPath( "base_path")
        self.control = CommandInterfaces( "PATH" )

        self.rate = rospy.Rate( 5 )

        self.status_mission = 0

        self.ok_count = 5

    def start_mission( self): # status_mission is 0

        self.control.publish_data( "START doing mission path" )        

        # Reset state
        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -1 )
        self.control.sleep()
        self.control.publish_data( "Wait depth" )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()  
        self.find_path()

    def find_path( self ):
        self.control.publish_data( "FIND doing mission to find" )

        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time ).to_sec() 

        self.control.deactivate( ['x' , 'y' ] )
        count = 0
        mode  = 1 # 1 : left 2 : right 3 : left
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.num_point != 0 ):
                self.control.get_state()
                count += 1
            else:
                count = 0

            if( count == 2 ):
                self.control.publish_data("!!!!!!!!! STRATEGY FIND PATH !!!!!!!!!!!!!" )
                target_depth = -1.0
                while( not rospy.is_shutdown() ):
                    self.rate.sleep()
                    self.vision.call_data()
                    self.vision.echo_data()

                    relative_x = 0
                    relative_y = 0
                    ok_x = False
                    ok_y = False
                    
                    if( self.vision.num_point == 0 ):
                        relative_y = 0
                    elif( self.vision.x_point[0] > 30 ):
                        relative_y = -1*PATH_FORCE_Y_
                    elif( self.vision.x_point[0] < -30 ):
                        relative_y = PATH_FORCE_Y_
                    else:
                        ok_y = True

                    if( self.vision.num_point == 0):
                        relative_x = 0
                    elif( self.vision.y_point[0] > 20 ):
                        relative_x = PATH_FORCE_X_
                    elif( self.vision.y_point[0] < -20 ):
                        relative_x = -1*PATH_FORCE_X_
                    else:
                        ok_x = True

                    if( ok_x and ok_y ):
                        self.control.force_xy( 0 , 0 )
                        if( self.control.check_z( 0.15 ) ):
                            if( target_depth < -2.4 ):
                                self.control.publish_data( "Breaking and go to setup point")
                                break
                            else:
                                target_depth -= 0.5
                                self.control.publish_data( "I command depth to " 
                                    + str( target_depth ) )
                                self.control.absolute_z( target_depth )
                    else:
                        self.control.publish_data( "FIND command " 
                            + repr(( relative_x , relative_y ) ) )
                        self.control.force_xy( relative_x , relative_y )

                self.control.force_xy( 0 , 0 )
                break

            diff_time = ( rospy.get_rostime() - start_time ).to_sec()
            if( mode == 1 ):
                self.control.force_xy( 0.1 , 1.2 )
                if( diff_time > PATH_FIND_TIME_ ):
                    self.control.publish_data( "FIND mode 1 time out")
                    mode = 2
                    start_time = rospy.get_rostime()
            elif( mode == 2 ):
                self.control.force_xy( 0.1 , -1.2 )
                if( diff_time > (PATH_FIND_TIME_ * 2.0) + 2 ):
                    self.control.publish_data( "FIND mode 2 time out")
                    mode = 3
                    start_time = rospy.get_rostime() 
            elif( mode == 3 ):
                self.control.force_xy( 0.1 , 1.2 )
                if( diff_time > ( PATH_FIND_TIME_ + 2 ) ):
                    self.control.publish_data( "FIND mode 3 time out")
                    mode = 4 
            else:
                self.control.force_false() 
                break

        self.control.activate( ['x' , 'y'])

        result = False
        if( count == 2 ):
            self.setup_point()
            result = True
        else:
            self.control.publish_data( "FIND Don't found path")
            result = False

        return result

    def setup_point( self ): # status_mission = 2

        self.control.publish_data( "SETUP Move to first point" )
        self.control.deactivate( ['x' , 'y'] ) 

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()
        
            relative_x = 0
            relative_y = 0
            ok_x = False
            ok_y = False
            
            if( self.vision.num_point == 0 ):
                relative_y = 0
            elif( self.vision.x_point[0] > 20 ):
                relative_y = -PATH_FORCE_Y_
            elif( self.vision.x_point[0] < -20 ):
                relative_y = PATH_FORCE_Y_
            else:
                ok_y = True

            if( self.vision.num_point == 0):
                relative_x = 0
            elif( self.vision.y_point[0] > 20 ):
                relative_x = PATH_FORCE_X_
            elif( self.vision.y_point[0] < -20 ):
                relative_x = -1.0*PATH_FORCE_X_
            else:
                ok_x = True
        
            if( ok_x and ok_y ):
                self.control.publish_data( "SETUP xy are ok" )
                self.control.force_xy( 0 , 0 )
                break
            else:      
                self.control.publish_data("SETUP command force x : y === "
                    + str( relative_x ) + " : "
                    + str( relative_y ) )
                self.control.force_xy( relative_x , relative_y )

        self.control.publish_data( "SETUP I will deactivate yaw for rotation")
        self.control.deactivate( ['x' , 'y' , 'yaw' ] )
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            diff = zeabus_math.bound_radian( self.vision.rotation[0] - ( math.pi / 2 ) )
            self.control.publish_data( "SETUP diff yaw is " + str( diff ) )
            if( abs( diff ) < 0.1 ):
                self.control.force_false()
                self.control.publish_data( "Now rotation are ok" )
                self.control.sleep()
                self.control.activate( ['x' , 'y' ,'yaw'] )
                break
            elif( diff > 0 ):
                self.control.publish_data( "SETUP rotation left")
                self.control.force_xy_yaw( 0 , 0 , 0.3 )
            else:
                self.control.publish_data( "SETUP rotation right" )
                self.control.force_xy_yaw( 0 , 0 , -0.3 )

        self.control.publish_data( "SETUP Now rotation finish next is moving on path")
        self.control.deactivate( [ 'x' , 'y' ])

        self.moving_on_path()

    def moving_on_path( self ): # status_mission = 3

        # Move to center of path
        self.control.publish_data( "MOVING_ON_PATH move to center of point one" )
        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            force_x = 0
            force_y = 0
            target_point = 0
            if( self.vision.x_point[ 0 ] < -20  ):
                force_y = PATH_FORCE_Y_
            elif( self.vision.x_point[ 0 ] > 20 ):
                force_y = -1.0*PATH_FORCE_Y_
            elif( self.vision.y_point[ 0 ] < -20 ):
                force_x = -1.0*PATH_FORCE_X_
            elif( self.vision.y_point[ 0 ] > 20 ):
                force_x = PATH_FORCE_X_
            elif( not self.control.check_yaw( 0.12 ) ):
                self.control.publish_data( "MOVING_ON_PATH SECTION 1 now center Waiting yaw")
            else:
                self.control.publish_data( "MOVING_ON_PATH SECTION 1 now center move direct")
                start_time = rospy.get_rostime()
                while( (not rospy.is_shutdown() ) and self.vision.num_point != 3 ):
                    self.rate.sleep()
                    self.vision.echo_data()
                    self.vision.call_data()
                    self.control.force_xy( 1 , 0 )
                    self.control.publish_data( "MOVING_ON_PATH SECTION 1 move forward")
                self.control.force_xy( 1 , 0 ) 
                break
            self.control.publish_data( "MOVING_ON_PATH SECTION 1 command " 
                + repr( ( force_x , force_y )))
            self.control.force_xy( force_x , force_y )

        self.control.publish_data( "MOVING_ON_PATH SECTION move to center of point 2")

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            force_x = 0
            force_y = 0
            target_point = 0
            if( self.vision.x_point[ 1 ] < -40  ):
                force_y = PATH_FORCE_Y_
            elif( self.vision.x_point[ 1 ] > -20 ):
                force_y = -1.0*PATH_FORCE_Y_
            elif( self.vision.y_point[ 1 ] < -15 ):
                force_x = -1.0*PATH_FORCE_X_
            elif( self.vision.y_point[ 1 ] > 15 ):
                force_x = PATH_FORCE_X_
            else:
                self.control.publish_data( "MOVING_ON_PATH SECTION 2 now center of point 2")
                break
            self.control.publish_data( "MOVING_ON_PATH SECTION 2 command " 
                + repr( ( force_x , force_y )))
            self.control.force_xy( force_x , force_y )

        self.control.publish_data( "MOVING_ON_PATH I will deactivate yaw for rotation")
        self.control.deactivate( ['x' , 'y' , 'yaw' ] )
        while( not rospy.is_shutdown() ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            diff = zeabus_math.bound_radian( self.vision.rotation[ self.vision.num_point -2 ] 
                -  ( math.pi / 2 ) )
            self.control.publish_data( "MOVING_ON_PATH diff yaw is " + str( diff ) )
            if( abs( diff ) < 0.1 ):
                self.control.force_false()
                self.control.publish_data( "Now rotation are ok" )
                self.control.sleep()
                self.control.activate( ['x' , 'y' ,'yaw'] )
                self.control.relative_yaw( 0 , False )
                self.control.sleep()
                break
            elif( diff > 0 ):
                self.control.publish_data( "MOVING_ON_PATH rotation left")
                self.control.force_xy_yaw( 0 , 0 , 0.3 )
            else:
                self.control.publish_data( "MOVING_ON_PATH rotation right" )
                self.control.force_xy_yaw( 0 , 0 , -0.3 )

        self.control.publish_data( "MOVING_ON_PATH Now rotation finish next is moving on path")
        self.control.deactivate( [ 'x' , 'y' ])

        self.control.absolute_z( -1.5 )
        start_time = rospy.get_rostime()
        diff_time = ( rospy.get_rostime() - start_time).to_sec()
        while( not rospy.is_shutdown() and diff_time < PATH_LAST_TIME_ ):
            self.rate.sleep()
            self.control.force_xy( 1.2 , 0 )
            diff_time = ( rospy.get_rostime() - start_time).to_sec()

        self.control.activate( ['x' , 'y' ] )
        self.control.publish_data( "MOVING_ON_PATH finish yaw")

if __name__=="__main__" :
    rospy.init_node( "mission_path" )

    mission = Path( )
    mission.start_mission()
