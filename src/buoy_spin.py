#!/usr/bin/env python2
# FILE			: buoy_spin.py
# AUTHOR		: K.Supasan
# CREATE ON		: 2019, July 01 (UTC+0)
# MAINTAINER	: K.Supasan

# README
#   What is mean of name buoy_spin
#       buoy is mission buoy. I think you can guess it.
#       spin is meaning how to searching buoy mission. Yes! I will use rotation_yaw to find that
#   Rule of Decision
#       Vision have send value about score of buoy in range 0 - 100
#           30 -> I will remember have this point 
#           50 -> I will go that to check picture
#           70 -> I beleive don't care other data
#   What process of this mode?
#       status = 0 
#           This function will use only for prepare depth to doing mission
#       status = 1
#           Survey to find mission and have check point but not survey left right.
#           I use spin and move to check instead
#       

# REFERENCE

import rospy
import math
from zeabus.math import general as zeabus_math
from zeabus.control.command_interfaces import CommandInterfaces
from zeabus.vision.analysis_buoy import AnalysisBuoy

class Buoy:

    def __init__( self ):

        self.vision = AnalysisBuoy( "base_buoy" )
        self.control = CommandInterfaces( "BUOY" )

        self.rate = rospy.Rate( 5 )

        self.status_mission = 0

        self.ok_count = 5 

        self.start_yaw = 0

        self.level_score = [ 30 , 50 , 70 ] # don't care < 30 , save < 50 , check < 70 , do > 70

        # x y z roll pitch yaw
        self.start_point = [ 0 , 0 , 0 , 0 , 0 , 0]

        self.save_point_center = {
            'target_yaw' : 0
            ,'score'     : 0
            ,'x'         : 0 
        } 

        self.save_point_right = {
            'target_yaw' : 0
            ,'score'     : 0
            ,'x'         : 0 
        } 

        self.save_point_left = {
            'target_yaw' : 0
            ,'score'     : 0
            ,'x'         : 0 
        } 

    def start_mission( self ): # status_mission is 0
        self.control.publish_data( "Start doing mission buoy about searching" )

        self.control.reset_state( 0 , 0 )

        self.control.absolute_z( -2 )

        self.control.publish_data( "Wait depth")
        while( not self.control.check_z( 0.15 ) ):
            self.rate.sleep()

        self.relative_xy( 1 , 0 )
        
        self.find_buoy()

    def find_target( self ):

        self.control.publish_data( "Waiting and save target_yaw" )
        while( not self.control.check_yaw( 0.15 ) ):
            self.rate.sleep()

        self.update_target()
        self.start_yaw = self.target_pose[5];
        self.control.publish_data( "Start target yaw is " + str( self.start_yaw ) )
        self.save_point_center['target_yaw'] = self.start_yaw
        self.save_point_right['target_yaw'] = zeabus_math.bound_radian( self.start_yaw
            - ( math.pi / 4 ) )
        self.save_point_left['target_yaw'] = zeabus_math.bound_radian( self.start_yaw
            + ( math.pi / 4 ) )
        self.control.publish_data( "(left straight right)" 
            , repr( ( self.save_point_left['target_yaw'] 
                , self.save_point_center['target_yaw'] 
                , self.save_point_right['target_yaw'] ) ) )

        # 0 don't choose 1 left 2 center 3 right
        data_choose = 0

        while( not rospy.is_shutdown() ):
            self.rate.sleep()

            self.control.publish_data( "Waiting ok xy")
            while( not self.control.check_xy( 0.1 , 0.1 ) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting yaw")
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            # Hint this part check picture have line is 28 line 
            self.control.publish_data( "Try to find picture direct mode" )
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.found ):
                self.save_point_center['score'] = self.vision.score
                self.save_point_center['x'] = self.vision.analysis[ 'x' ]
            else:
                self.save_point_center['score'] = 0
                self.save_point_center['x'] = 0

            if( self.save_point_center['score'] > self.level_score[2] ):
                self.control.publish_data( "Break process find I found score over straight")
                data_choose = 2
                break
            elif( self.save_point_center['score'] > self.level_score[1] ):
                self.control.publish_data( "Break process find I have to check")
                result = self.check_target( self.save_point_center['x'] 
                    , self.save_point_center['target_yaw'])
                if( result ):
                    data_choose = 2
                    break
                else:
                    self.control.publish_data( "It not think remove this point" )
                    self.save_point_center['score'] = 0
                    self.save_point_center['x'] = 0
                    continue
            else:
                self.control.publish_data( "Center range don't found anything")

            self.control.publish_data( "Next I will find picture left spin 45 degree" )
            self.control.absolute_yaw( self.save_point_left['target_yaw'] )
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            # Hint this part check picture have line is 28 line 
            self.control.publish_data( "Try to find picture left mode" )
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.found ):
                self.save_point_left['score'] = self.vision.score
                self.save_point_left['x'] = self.vision.analysis[ 'x' ]
            else:
                self.save_point_left['score'] = 0
                self.save_point_left['x'] = 0

            if( self.save_point_left['score'] > self.level_score[2] ):
                self.control.publish_data( "Break process find I found score over left")
                data_choose = 1
                break
            elif( self.save_point_left['score'] > self.level_score[1] ):
                self.control.publish_data( "Break process find I have to check")
                result = self.check_target( self.save_point_left['x'] 
                    , self.save_point_left['target_yaw'])
                if( result ):
                    data_choose = 2
                    break
                else:
                    self.control.publish_data( "It not think remove this point" )
                    self.save_point_left['score'] = 0
                    self.save_point_left['x'] = 0
                    continue
            else:
                self.control.publish_data( "Left range don't found anything")

            self.control.publish_data( "Next I will find picture left spin 45 degree" )
            self.control.absolute_yaw( self.save_point_right['target_yaw'] )
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            # Hint this part check picture have line is 28 line 
            self.control.publish_data( "Try to find picture right mode" )
            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.found ):
                self.save_point_right['score'] = self.vision.score
                self.save_point_right['x'] = self.vision.analysis[ 'x' ]
            else:
                self.save_point_right['score'] = 0
                self.save_point_right['x'] = 0

            if( self.save_point_right['score'] > self.level_score[2] ):
                self.control.publish_data( "Break process find I found score over right")
                data_choose = 3
                break
            elif( self.save_point_right['score'] > self.level_score[1] ):
                self.control.publish_data( "Break process find I have to check")
                result = self.check_target( self.save_point_right['x'] 
                    , self.save_point_right['target_yaw'])
                if( result ):
                    data_choose = 2
                    break
                else:
                    self.control.publish_data( "It not think remove this point" )
                    self.save_point_right['score'] = 0
                    self.save_point_right['x'] = 0
                    continue
            else:
                self.control.publish_data( "Right range don't found anything")

            self.control.publish_data( "This round never check I will search storage" )
            self.control.publish_data( "List score : " + repr( ( self.save_point_left['score']
                , self.save_point_center['score'] 
                , self.save_point_right['score'] ) ) )

            if( ( self.save_point_left['score'] > self.save_point_center['score'] ) 
                    and ( self.save_point_left['score'] > self.save_point_right[ 'score'] ) ):
                if( self.save_point_left['score'] > self.level_score[1] ):
                    self.control.publish_data( "Left point are winner")
                    result = self.check_target( self.save_point_left['x'] 
                        , self.save_point_left['target_yaw'] )
                    if( result ):
                        data_choose = 2
                        break
            elif( self.save_point_center['score'] > self.save_point_right['score'] ):
                if( self.save_point_center['score'] > self.level_score[1] ):
                    self.control.publish_data( "Center point are winner" )
                    result = self.check_target( self.save_point_center['x']
                        , self.save_point_center['target_yaw'] )
                    if( result ):
                        data_choose = 2
                        break
            elif( self.save_point_right['score'] > self.level_score[1] ):
                self.control.publish_data( "Right point are winner" )
                result = self.check_target( self.save_point_right['x']
                    , self.save_point_center['target_yaw'] )
                if( result ):
                    data_choose = 2
                    break
            else:
                self.control.publish_data( "Don't choose anything move forward in start yaw")
                self.control.absolute_yaw( self.start_yaw )
                self.control.relative_xy( 1 , 0 )

        # data_choose 1 is left 2 is center 3 is right
        self.control.publish_data( "Finish loop find target, set yaw and data is " 
            + str(data_choose) )
        diff_yaw = 0
        temp_x = 0
        temp_y = 0
        self.control.absolute_yaw( self.start_yaw )
        if( data_choose == 1 ):
            temp_y = ( distance * math.cos( math.pi / -0.5 ) )
            temp_x = ( distance * meth.sin( diff_yaw / -0.5 ) / 2)
        elif( data_choose == 2 ):
        elif( data_choose == 3 ):
        else:
            self.control.publish_data( "Aborted mission" )

        if( data_choose in [ 1, 2, 3 ] ):
            self.lock_target()    

    def lock_target( self ): # status_mission = 2
        self.control.publish_data( "We have lock target move until don't found picture" )

        unfound = 0

        dash_distance = 0

        while( not rospy.is_shutdown() ): 

            self.control.publish_data( "Waiting z" )
            while( not self.control.check_z( 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting xy" )
            while( not self.control.check_xy( 0.15 , 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting yaw" )
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.found ):
                unfound = 0

                self.vision.call_data()
                self.vision.echo_data()
                if( self.vision.found ):
                    dash_distance = self.vision.analysis['x'] 
                    if( ( abs( self.vision.analysis['z'] ) > 20 ) 
                            and ( abs( self.vision.analysis['y'] ) > 30 ) ):
                        self.control.publish_data( "tune center move x and y" 
                            + repr( self.vision.analysis['z'] , self.vision.analysis['y'] ) )
                        self.relative_z( self.vision.analysis['z'] / 100 )
                        self.relative_xy( 0 , self.vision.analysis['y'] / 100 )
                    elif( abs( self.vision.analysis['z'] ) > 20 ):
                        self.control.publish_data( "tune center z " 
                            + str( self.vision.analysis['z'] ) )
                        self.relative_z( self.vision.analysis['z'] / 100 )
                    elif( abs( self.vision.analysis['y'] ) > 30 ):
                        self.control.publish_data( "tune center y " 
                            + str( self.vision.analysis['y'] ) )
                        self.relative_xy( 0 , self.vision.analysis['y'] / 100 )
                    else:
                        if( self.vision.analysis['x'] < 200 ):
                            dash_distance = self.vision.analysis['x']
                            self.control.publish_data( "Time to dash now " + str( dash_distance))
                            break
                        else:
                            self.control.publish_data( "Tune distance x " 
                                + str( self.vision.analysis['x'] ) )
                            self.relative_xy( 0 , self.vision.analysis['x'] / 150 )
                            dash_distance -= self.vision.analysis['x'] / 150
                else:
                    unfound += 1
                    if( unfound == 2 ):
                        self.control.publish_data( "Picture invisible dash now")

        self.dash_mode( dash_distance )

    def dash_mode( self , distance ):
        pass
 
    def check_target( self , distance , yaw ):
        self.control.publish_data( "Save point before move to check")
        self.control.absolute_yaw( self.start_yaw )
        self.control.update_target()
        self.start_point = self.control.target_pose

        diff_yaw = zeabus_math.bound_radian( yaw - self.start_yaw )
        temp_y = ( distance * math.cos( diff_yaw ) )
        temp_x = ( distance * meth.sin( diff_yaw ) / 2)
        self.control.publish_data( "movement to check ( x , y ) : " + repr( ( temp_x , temp_y ) )
        self.control.relative_xy( temp_x , temp_y )

        self.control.publish_data( "In check_target mode I will start to check picture")

        result = False

        unfound = 0
             
        while( not rospy.is_shutdown() ): 
            self.rate.sleep()

            self.control.publish_data( "Waiting z" )
            while( not self.control.check_z( 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting xy" )
            while( not self.control.check_xy( 0.15 , 0.15 ) ):
                self.rate.sleep()

            self.control.publish_data( "Waiting yaw" )
            while( not self.control.check_yaw( 0.15 ) ):
                self.rate.sleep()

            self.vision.call_data()
            self.vision.echo_data()
            if( self.vision.found ):
                temp_ok = 0
                unfound = 0

                if( abs( self.vision.center_y ) > 30 ):
                    self.control.publish_data( "Move relative z value is " 
                        + str( self.vision.analysis[ 'z' ] / 200 ) )
                    self.control.relative_z( self.vision.analysis[ 'z' ] / 200 )
                else:
                    temp_ok += 1

                if( abs( self.vision.center_x ) > 40 )
                    self.control.publish_data( "Survey relative y value is " 
                        + str( self.vision.analysis[ 'y' ] / 150 ) )
                    self.control.relative_xy( 0.1 , self.vision.analysis[ 'y' ] / 150 )
                else:
                    temp_ok += 1

                if( temp_ok == 2):
                    if( self.vision.score > self.level_score[2] ):
                        self.control.publish_data( "Score are over max lock target" )
                        result = True
                        break
                    else:
                        self.control.publish_data( "Move forward for adding score")
                        self.control.relative_xy( 0.5 , 0)
            else:
                self.control.publish_data( "Warning Unfound target")
                unfound += 1
                if( unfound == 2 ):
                    break

        if( ! result ):
            self.control.publish_data( "Return to pose : " + repr( self.start_point ) )
            self.control.absolute_xy( self.start_point[0] ,self.start_point[1] )
            self.control.absolute_yaw( self.start_yaw )
            self.control.absolute_z( self.start_point[2] )
            self.control.publish_data( "Move forward from checkpoint 1 meter")
            self.control.relative_xy( 1 , 0 , True , True )
        return result 

        
if __name__=="__main__" :
    rospy.init_node( "mission_buoy" )

    mission = Buoy()
    mission.start_mission() 
