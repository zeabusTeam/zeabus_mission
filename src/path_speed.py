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
from mission_constant import *

class Path:

    def __init__( self , move_x , move_y):

        self.vision = AnalysisPath( "base_path")
        self.control = CommandInterfaces( "PATH" )

        self.rate = rospy.Rate( 5 )

        self.status_mission = 0

        self.ok_count = 5

        self.move_x = move_x
        self.move_y = move_y 

    def start_mission( self): # status_mission is 0

        self.control.publish_data( "Start doing mission path" )        

        # Reset state
        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -1 )

        self.control.publish_data( "Wait depth" )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()  
        self.find_path()

    def find_path( self ): # status_mission is 1

        # In function find_path we move to triangle pattern to find path 
        # You can think I will survey around area of rectangle

        self.control.absolute_z( -1 )
        self.control.publish_data( "We have go to depth 1.5 meters")
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        result = False
#        round_move = 0
        round_move = 6
        absolute_z = -1.6

        while( not rospy.is_shutdown() ):
            round_move += 1

            count_found = 0
            self.control.publish_data( "Waiting ok xy and search duration move" )
            while( ( not self.control.check_xy( 0.12, 0.12 ) ) and count_found < 3):
                self.rate.sleep()
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.num_point != 0 ):
                    self.control.publish_data( "Vision found have to stop move and check" )
                    self.control.update_target()
                    save_point = ( self.control.target_pose[0] , self.control.target_pose[1] )
                    self.control.publish_data( "Save point is " + repr( save_point ) )
                    self.control.relative_xy( self.vision.y_point[0] * 0.5 / 100 
                        , self.vision.x_point[0] * -0.8 / 100 )
                    count_found += 1
                    while( not rospy.is_shutdown() and count_found < 3 ):
                        self.rate.sleep()
                        self.vision.call_data()
                        self.vision.echo_data()
                        if( self.vision.num_point != 0 ):
                            count_found += 1
                        else:
                            self.control.absolute_xy( save_point[0] , save_point[1])
                            count_found = 0
                            break
                else:
                    self.count_found = 0
                            
            if( count_found == 0 ):
                self.control.publish_data( "Try to find picture when alive destination")
            while( ( not rospy.is_shutdown() ) and count_found < 3):
                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.num_point != 0 ):
                    count_found += 1
                else:
                    count_found = 0
                    break
                self.rate.sleep()

            if( count_found == 3 ):
                self.control.publish_data("We found path, so go depth and center of first point")
                self.control.deactivate( ['x' , 'y' ] ) 
                target_depth = -1
                while( ( not rospy.is_shutdown() ) and target_depth > -2.5):
                    self.rate.sleep()
                    force_x = 0
                    force_y = 0
                    self.vision.call_data() 
                    self.vision.echo_data()
                    if( self.vision.x_point[ 0 ] < -20  ):
                        force_y = PATH_FORCE_Y 
                    elif( self.vision.x_point[ 0 ] > 20 ):
                        force_y = -1*PATH_FORCE_Y
                    else:
                        None
                    if( self.vision.y_point[ 0 ] < -20 ):
                        force_x = -PATH_FORCE_X
                    elif( self.vision.y_point[ 0 ] > 20 ):
                        force_x = PATH_FORCE_X
                    else:
                        None
                    self.control.publish_data( "FIND_PATH command force_xy : " 
                        + repr( ( force_x , force_y ) ) )
                    self.control.force_xy( force_x , force_y )

                    if( self.control.check_z( 0.15 ) ):
                        self.control.absolute_z( target_depth ) 
                        target_depth -= 1
                self.control.publish_data( "FIRST POINT NOW CENTER")
                self.control.force_xy( 0 , 0 )
                self.status_mission = 2 
                break 
            else:
                relative_x = 0
                relative_y = 0
                if( round_move == 1 ):
                    relative_x = -1
                    relative_y = +1
                elif( round_move == 2 ):
                    relative_x = +3 
                    relative_y = 0
                elif( round_move == 3 ):
                    relative_x = +0 
                    relative_y = -2
                elif( round_move == 4 ):
                    relative_x = -3
                    relative_y = 0
                elif( round_move == 5 ):
                    relative_x = +1
                    relative_y = +1
                else:
                    self.control.publish_data( "Don't found target abort part mission" )
                    break
                self.control.publish_data( "Don't found move (x , y ) : {:6.3f} {:6.3f}".format( 
                     relative_x , relative_y ) )
                self.control.relative_xy( relative_x , relative_y )
            
        if( self.status_mission == 2 ):
            self.setup_point()
            result = True

        return result 

    def setup_point( self ): # status_mission = 2

        self.control.publish_data( "Now I move first point to picutre" )
        self.control.deactivate( ['x' , 'y'] ) 

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()

            self.control.publish_data( "I waiting yaw")
            while( not self.control.check_yaw( 0.10 ) ):
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
                relative_y = -PATH_FORCE_Y
            elif( self.vision.x_point[0] < -20 ):
                relative_y = PATH_FORCE_Y
            else:
                ok_y = True

            if( self.vision.num_point == 0):
                relative_x = 0
            elif( self.vision.y_point[0] > 20 ):
                relative_x = PATH_FORCE_X
            elif( self.vision.y_point[0] < -20 ):
                relative_x = -PATH_FORCE_X
            else:
                ok_x = True
        
            if( ok_x and ok_y ):
                self.control.publish_data( "xy is ok next I will collect point" )
                self.control.force_xy( 0 , 0 )
                break
            else:      
                self.control.publish_data("T will command force x : y "
                    + str( relative_x ) + " : "
                    + str( relative_y ) )
                self.control.force_xy( relative_x , relative_y )

        count = 0
        temp_yaw = []
        while( not rospy.is_shutdown() and count < 1):
            self.rate.sleep()
            self.control.publish_data( "Waiting yaw for calculate" )
            while( not self.control.check_yaw( 0.1 ) ):
                self.rate.sleep()
            self.control.publish_data( "Calculate yaw" )
            self.vision.call_data()
            self.vision.echo_data()   
            temp_yaw.append( 
                self.control.current_pose[5] 
                - ( ( math.pi / 2 ) 
                    - self.vision.rotation[0] ) )
            count += 1
            self.control.publish_data("temp_yaw " + repr( temp_yaw ) )
            self.status_mission = 3

        self.first_yaw = zeabus_math.bound_radian( sum( temp_yaw ) / 1 )
        self.control.publish_data( "Now I will absolute yaw is " + str( self.first_yaw ) )
        self.control.absolute_yaw( self.first_yaw )
        self.moving_on_path()

    def moving_on_path( self ): # status_mission = 3

        # Move to center of path
        self.control.publish_data( "MOVING_ON_PATH move to center first point" )
        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            force_x = 0
            force_y = 0
            target_point = 0
            if( self.vision.x_point[ 0 ] < -20  ):
                force_y = PATH_FORCE_Y
            elif( self.vision.x_point[ 0 ] > 20 ):
                force_y = -1.0*PATH_FORCE_Y
            elif( self.vision.y_point[ 0 ] < -20 ):
                force_x = -1*PATH_FORCE_X
            elif( self.vision.y_point[ 0 ] > 20 ):
                force_x = PATH_FORCE_X
            elif( not self.control.check_yaw( 0.12 ) ):
                self.control.publish_data( "MOVING_ON_PATH SECTION 1 now center Waiting yaw")
            else:
                self.control.publish_data( "MOVING_ON_PATH SECTION 1 now center move direct")
                start_time = rospy.get_rostime()
                diff = (rospy.get_rostime() - start_time).to_sec()
                while( (not rospy.is_shutdown() ) and diff < PATH_PASS_TIME ):
                    self.rate.sleep()
                    self.control.force_xy( 2 , 0 ) 
                    diff = (rospy.get_rostime() - start_time).to_sec()
                self.control.force_xy( 0 , 0 )
                break
            self.control.publish_data( "MOVING_ON_PATH SECTION 1 command " 
                + repr( ( force_x , force_y )))
            self.control.force_xy( force_x , force_y )

        self.control.force_xy( 0 , 0 )
        self.control.publish_data( "I want data to dicision want target I have to use")
        self.vision.call_data()
        self.vision.echo_data()

        if( self.vision.rotation[ self.vision.num_point - 2 ] < math.pi/2 ):
            self.control.publish_data( "rotation right" )       
            self.control.relative_yaw( math.pi / -4 , True )
        else:
            self.control.publish_data( "rotation left" )       
            self.control.relative_yaw( math.pi / 4 , True )

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()
            self.vision.call_data()
            self.vision.echo_data()
            force_x = 0
            force_y = 0
            target_point = 0
            if( self.vision.x_point[ self.vision.num_point-2 ] < -20  ):
                force_y = PATH_FORCE_Y
            elif( self.vision.x_point[ self.vision.num_point-2 ] > 20 ):
                force_y = -1.0*PATH_FORCE_Y
            elif( self.vision.y_point[ self.vision.num_point-2 ] < -20 ):
                force_x = -1*PATH_FORCE_X
            elif( self.vision.y_point[ self.vision.num_point-2 ] > 20 ):
                force_x = PATH_FORCE_X
            elif( not self.control.check_yaw( 0.12 ) ):
                self.control.publish_data( "MOVING_ON_PATH SECTION 2 now center Waiting yaw")
            else:
                self.control.publish_data( "MOVING_ON_PATH SECTION 2 now center move direct")
                start_time = rospy.get_rostime()
                diff = (rospy.get_rostime() - start_time).to_sec()
                while( (not rospy.is_shutdown() ) and diff < ( PATH_PASS_TIME + 3 ) ):
                    self.rate.sleep()
                    self.control.force_xy( 2 , 0 ) 
                    diff = (rospy.get_rostime() - start_time).to_sec()
                self.control.force_xy( 0 , 0 )
                break
            self.control.publish_data( "MOVING_ON_PATH SECTION 2 command " 
                + repr( ( force_x , force_y ) ) )
            self.control.force_xy( force_x , force_y )

        self.control.activate([ 'x' , 'y' ])
        self.control.publish_data( "I'm finish")
        self.control.absolute_z( -1.5 )

if __name__=="__main__" :
    rospy.init_node( "mission_path" )

    mission = Path( 0 , 0)
    mission.start_mission()
