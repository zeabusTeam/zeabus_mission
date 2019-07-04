#!/usr/bin/env python2
# FILE			: path.py
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
        # You can think I will survey around area of triangle
        #               p4
        #              /  \
        #             /    \
        #            /  p1  \
        #           /        \
        #          /          \
        #         p2----------p3
        # p1 -> p2 move ( -1 , +1 ) # round_move = 1 
        # p2 -> p4 move ( +2 , -1 ) # round_move = 2
        # p4 -> p3 move ( -2 , -1 ) # round_move = 3
        # p3 -> p1 move ( +1 , +1 ) # round_move = 4
        # round_move 5 is check and end

        self.control.absolute_z( -1 )
        self.control.publish_data( "We have go to depth 1.5 meters")
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        result = False
        round_move = 0

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
                temp_x = self.vision.y_point[0] * 0.5 / 100
                temp_y = -1 * self.vision.x_point[0] * 0.8 / 100
                self.control.publish_data( "I found path and move xy --> " 
                    + str( temp_x ) + " : "
                    + str( temp_y ) )
                self.control.relative_xy( temp_x , temp_y )
                while( not self.control.check_xy( 0.15 , 0.15 ) ):
                    self.rate.sleep()
                self.control.publish_data( "I found path and will go depth" )
                self.control.absolute_z( -2.5 )
                while( not self.control.check_z( 0.15 ) ):
                    self.rate.sleep()
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

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()

            self.control.publish_data( "I waiting xy")
            while( not self.control.check_xy( 0.1, 0.1 ) ):
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
                relative_y = -0.15
            elif( self.vision.x_point[0] < -20 ):
                relative_y = +0.15
            else:
                ok_y = True

            if( self.vision.num_point == 0):
                relative_x = 0.25
            elif( self.vision.y_point[0] > 20 ):
                relative_x = 0.15
            elif( self.vision.y_point[0] < -20 ):
                relative_x = -0.15
            else:
                ok_x = True
        
            if( ok_x and ok_y ):
                self.control.publish_data( "xy is ok next I will collect point" )
                break
            else:      
                self.control.publish_data("T will move x : y "
                    + str( relative_x ) + " : "
                    + str( relative_y ) )
                self.control.relative_xy( relative_x , relative_y )

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
        self.control.publish_data( "Now waiting yaw are ok" )
        count = 0 
        while( ( not rospy.is_shutdown() ) and count < self.ok_count ):
            if( self.control.check_yaw( 0.1 ) ):
                count += 1
            else:
                count = 0 
            self.rate.sleep()

        self.control.relative_xy( 0.60 , 0 )
        self.control.publish_data( "Now waiting xy are ok")
        count = 0 
        while( ( not rospy.is_shutdown() ) and count < self.ok_count ):
            if( self.control.check_xy( 0.1 , 0.1 ) ):
                count += 1
            else:
                count = 0 
            self.rate.sleep()

        self.control.publish_data( "I want data to dicision want target I have to use")
        self.vision.call_data()
        self.vision.echo_data()

        if( self.vision.rotation[ self.vision.num_point - 2 ] < math.pi/2 ):
            self.control.publish_data( "rotation right" )       
            self.control.relative_yaw( math.pi / -4 , True )
        else:
            self.control.publish_data( "rotation left" )       
            self.control.relative_yaw( math.pi / 4 , True )

        self.control.publish_data( "Now waiting yaw are ok")
        count = 0 
        while( ( not rospy.is_shutdown() ) and count < self.ok_count ):
            if( self.control.check_yaw( 0.1 ) ):
                count += 1
            else:
                count = 0 
            self.rate.sleep()

        self.control.relative_xy( 1 , 0 )
        self.control.publish_data( "Now waiting xy are ok")
        count = 0 
        while( ( not rospy.is_shutdown() ) and count < self.ok_count ):
            if( self.control.check_xy( 0.1 , 0.1 ) ):
                count += 1
            else:
                count = 0 
            self.rate.sleep()

        self.control.publish_data( "I'm finish")
        self.control.absolute_z( -1.5 )

if __name__=="__main__" :
    rospy.init_node( "mission_path" )

    mission = Path()
    mission.start_mission()
