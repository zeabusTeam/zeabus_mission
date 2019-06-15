#!/usr/bin/python2
from __future__ import print_function
import rospy
from control_lib import Control

control = Control('Square')

def main () :
    print ("GO GO RIGHT")
    start_position = control.getCurrent()
    control.moveDist([25,0,0,0,0,0])
    while control.isOkay(0.05) == False:
        continue
    print ("GO GO FORWARD")
    control.moveDist([0,-50,0,0,0,0])
    while control.isOkay(0.05) == False:
        continue
    '''
    print ("GO GO LEFT") 
    control.moveDist([0,-3,0,0,0,0])
    while control.isOkay(0.05) == False:
        continue
    print ("GO GO BACK")
    control.moveDist([-3,0,0,0,0,0])
    while control.isOkay(0.05) == False:
        continue
    print ("GO GO TO START")
    control.moveDist([1.5,0,0,0,0,0])
    while control.isOkay(0.05) == False:
        continue
    print ("TURN AROUND")
    control.moveDist([0,0,0,0,0,180])
    while control.isOkay(0.05) == False:
        continue
    '''
    print ("FINISH FINISH FINISH")
if __name__ == "__main__" :
    rospy.init_node('square',anonymous=False)
    main()
    
    
