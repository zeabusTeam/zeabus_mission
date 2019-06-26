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

    def __init__( self ):

        self.vision = AnalysisPath()
        self.control = CommandInterfaces( "GAP" )

        self.rate = rospy.Rate( 10 )

        self.status_mission = 0

    def start_mission( self ): # status_mission is 0

        self.control.publish( "Start doing message path" )        

        # Reset state
        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -2 )

        self.control.publish( "Wait depth" )
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()  

    def find_path( self ): # status_mission is 1

        while( not rospy.is_shutdown() ):

            self.control.publish( "Waiting message xy" )
            while( not self.control.check_xy( 0.1, 0.1 ) ):
                self.rate.sleep()

            self.control.publish( "Try to find pictur")
            count_found = 0
            while( ( not rospy.is_shutdown() ) and count_found < 3):
                if( self.vision.call_data() ):
                    self.vision.echo_data()
                    count_found += 1
                else:
                    break
                self.rate.sleep()

            if( count_found == 3 ):
                self.control.publish( "I found path and will go depth" )
                self.control.absolute_z( -3.0 )
                while( not self.control.check_z( 0.15 ) ):
                    self.rate.sleep()
                self.status_mission = 2
                break
            else:
                self.control.publish( "Don't found move forward continous search" )
                self.relative_xy( 0.5 )
            
        if( self.status_mission = 2 ):
            self.setup_point()

    def setup_point( self ): # status_mission = 2

        self.control.publish( "Now I move first point to picutre" )

        while( ( not rospy.is_shutdown() ) ):
            self.rate.sleep()

            self.control.publish( "I waiting xy")
            while( not self.control.check_xy( 0.10, 0.1 ) ):
                self.rate.sleep()
            
            self.control.publish( "I waiting yaw")
            while( not self.control.check_yaw( 0.10 ) ):
                self.rate.sleep()
            
            self.vision.call_data()
            self.vision.echo_data()
        
            relative_x = 0
            relative_y = 0
            ok_x = False
            ok_y = False

            if( self.vision.x_point[0]  > 20 ):
                relative_y = 0.2
            elif( self.vision.x_point[0] < -20 ):
                relative_y = -0.2
            else:
                ok_y = True

            if( self.vision.y_point[0]  > 20 ):
                relative_x = 0.2
            elif( self.vision.y_point[0] < -20 ):
                relative_x = -0.2
            else:
                ok_x = True
        
            if( ok_x and ok_y ):
                self.control.publish( "xy is ok next I will collect point" )
                break
            else:      
                self.control.publish("T will move x : y "
                    + str( relative_x ) + " : "
                    + str( relative_y ) )

        count = 0
        temp_yaw = []
        while( not rospy.is_shutdown() and count < 5):
            self.rate.sleep()
            self.control.get_state()
            self.vision.call_data()
            self.vision.echo_data()   
            temp_yaw.append( 
                self.control.current_pose[5] 
                - ( ( math.pi / 2 ) 
                    - self.vision.rotation[0] ) )
            count += 1
            self.control.publish("temp_yaw " + repr( temp_yaw ) )
            self.status_mission = 3

        self.first_yaw = zeabus_math.bound_radian( sum( temp_yaw ) / 5 )
        self.control.publish( "Now I will absolute yaw is " + str( self.first_yaw ) )
        self.control.absolute_yaw( self.first_yaw )
        self.moving_on_path()

    def moving_on_path( self ): # status_mission = 3
        self.control.publish( "Now waiting yaw are ok" )
        while( not self.control.check_yaw(0.1) ):
            self.rate.sleep()

        self.relative_xy( 0.7 , 0 )
        self.control.publish( "Now waiting xy are ok")
        while( not self.control.check_xy( 0.1 , 0.1 ) ):
            self.rate.sleep()

        self.relative_yaw( math.pi / -4 , True )
        self.control.publish( "Now waiting yaw are ok")
        while( not self.control.check_yaw( 0.1 ) ):
            self.rate.sleep()

        self.relative_xy( 1 , 0 )
        self.control.publish( "Now waiting xy are ok")
        while( not self.control.check_xy( 0.1 , 0.1 ) ):
            self.rate.sleep()

        self.control.publish( "I'm finish")
        self.control.absolute_z( -1.5 )

if __name__=="__main__" :
    rospy.init_node( "mission_path" )

    mission = Path()
    mission.start_mission()
