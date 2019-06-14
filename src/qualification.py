#!/usr/bin/python2
from __future__ import print_function
import rospy
from zeabus_utility.srv import VisionSrvQualification
from std_msgs.msg import String
from control_lib import Control

# control = Control('Qualify')

def main():
    service_name = 'vision/qualification'
    print('wait service')
    rospy.wait_for_service(service_name)
    print('service start')
    call = rospy.ServiceProxy(service_name, VisionSrvQualification)
    i = 0
    last = 0
    last_move = False
    #control.moveDist([0,0,-1.2,0,0,0])
    # while control.isOkey == False :
        # continue
    while not rospy.is_shutdown():
        # current = control.getCurrent()
        try:
            res = call(String('qualification'),String('gate'))
            print (res.data.point1)
            print (res.data.point2)
            print (res.data.area)
        except:
            if i is not 0:
                print('wait service')
            i = 0
        area = res.data.area.data
        if last is not -1 and res.data.type is -1:
            print('Image is none...')
        elif res.data.type.data >= 0:
            i += 1
            print('Calling {} times'.format(i))
            if res.data.type.data == 0 :
                print ("Not Find Moving Forward")
                # control.moveDist([1,0,0,0,0,0])
                # while control.isOkey == False :
                    # continue
            elif res.data.type.data > 0 and not last_move:
                while area <= 0.85 :
                    area = res.data.area.data
                    print ("area = " + str(area))
                    cx = (res.data.point1.x+res.data.point2.x)/2
                    print ("cx = " + str(cx))
                    res = call(String('qualification'),String('gate'))
                    if cx <= -0.1 :
                        print ("Moving Left")
                        # control.moveDist([0,-0.1,0,0,0,0])
                        # while control.isOkey == False :
                            # continue
                    elif cx >= 0.1 :
                        print ("Moving Right")
                        # control.moveDist([0,0.1,0,0,0,0])
                        # while control.isOkey == False :
                            # continue
                    else :
                        print ("Moving Forward")
                        #control.moveDist([0.1,0,0,0,0,0])
                        # while control.isOkey == False :
                            # continue
                    rospy.sleep(0.5)
                print ("Last Move")
                last_move = True
                #control.moveDist([2,0,0,0,0,0])
                # while control.isOkey == False :
                    # continue
        last = res.data.type
        # if last_move :
        rospy.sleep(0.5)

if __name__ == "__main__":
    main()
