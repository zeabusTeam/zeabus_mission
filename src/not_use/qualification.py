#!/usr/bin/python2
from __future__ import print_function
import rospy
from zeabus_utility.srv import VisionSrvQualification
from std_msgs.msg import String
from control_lib import Control

control = Control('Qualify')

def main():
    service_name = 'vision/qualification'
    print('wait service')
    rospy.wait_for_service(service_name)
    print('service start')
    call = rospy.ServiceProxy(service_name, VisionSrvQualification)
    i = 0
    last = 0
    last_move = False 
    finish_marker = False
    old_cx = 0.5
    control.moveDist([0,0,-1.2,0,0,0])
    while control.isOkay(0.05) == False :
        continue
    while not rospy.is_shutdown():
        current = control.getCurrent()
        try:
            res = call(String('qualification'),String('gate'))
            print (res.data.point1)
            print (res.data.point2)
            print (res.data.area)
        except:
            if i is not 0:
                print('wait service')
            i = 0
        if not last_move :
            print ("---------------- Finding Gate --------------")
            area = res.data.area.data
            if last is not -1 and res.data.type.data is -1:
                print('Image is none...')
            elif res.data.type.data >= 0:
                i += 1
                print('Calling {} times'.format(i))
                if res.data.type.data == 0 :
                    print ("Not Find Moving Forward")
                    control.moveDist([0,1,0,0,0,0])
                    while control.isOkay(0.05) == False :
                        continue
                elif res.data.type.data > 0 :
                    while area <= 0.85 :
                        print ("---------------- Finding Gate --------------")
                        res = call(String('qualification'),String('gate'))
                        area = res.data.area.data
                        print ("area = " + str(area))
                        cx = (res.data.point1.x+res.data.point2.x)/2
                        print ("cx = " + str(cx))
                        if cx <= -0.1 and old_cx - cx <= 0.1:
                            print ("Moving Left")
                            control.moveDist([0,-0.1,0,0,0,0])
                            while control.isOkay(0.05) == False :
                                continue
                        elif cx >= 0.1 and old_cx - cx <= 0.1:
                            print ("Moving Right")
                            control.moveDist([0,0.1,0,0,0,0])
                            while control.isOkay(0.05) == False :
                                continue
                        else :
                            print ("Moving Forward")
                            control.moveDist([0.1,0,0,0,0,0])
                            while control.isOkay(0.05) == False :
                                continue
                        old_cx = cx
                    # rospy.sleep(0.5)
                    print ("Last Move")
                    last_move = True
                    control.moveDist([2,0,0,0,0,0])
                    while control.isOkay(0.05) == False :
                        continue
        elif not finish_marker and last_move :
            print ("---------------- Finding Marker --------------")
            res = call(String('qualification'),String('marker'))
            if last is not -1 and res.data.type.data is -1:
                print('Image is none...')
            elif res.data.type.data != -1 :
                i += 1
                area = res.data.area.data
                print('Calling {} times'.format(i))
                if res.data.type.data == 0 :
                    print ("Not Find Moving Forward")
                    control.moveDist([1,0,0,0,0,0])
                    while control.isOkay() == False :
                        continue
                #else :
                #if True :
                elif res.data.type.data > 0 :
                    while area <= 0.2:
                        print ("---------------- Finding Marker --------------")
                        print ("    ---------------- Step 1 ---------------")
                        res = call(String('qualification'),String('marker'))
                        cx = (res.data.point1.x+res.data.point2.x)/2
                        print ("cx = " + str(cx))
                        if cx <= -0.1 :
                            print ("Moving Left")
                            # control.moveDist([0,0.1,0,0,0,0])
                            # while control.isOkey() == False :
                                # continue
                        elif cx >= 0.1 :
                            print ("Moving Right")
                            # control.moveDist([0,-0.1,0,0,0,0])
                            # while control.isOkey() == False :
                                # continue
                        else :
                            print ("Moving Forward")
                            #control.moveDist([0.1,0,0,0,0,0])
                            # while control.isOkey() == False :
                                # continue
                            # continue
                    # front_marker = control.getCurrent()
                    while res.data.point2.x != -1 :
                        print ("---------------- Finding Marker --------------")
                        print ("    ---------------- Step 2 ---------------")
                        res = call(String('qualification'),String('marker'))
                        area = res.data.area.data
                        print ("Moving Right")
                        #control.moveDist([0,-0.1,0,0,0,0])
                        # while control.isOkey() == False :
                            # continue
                    # right_marker = control.getCurrent(
                    print ("    ---------------- Step 3 ---------------")  
                    print ("Moving Left")                                          
                    #control.moveDist([0,2*(right_marker[1]-front_marker[1]),0,0,0,0])
                    # while control.isOkey() == False :
                        # continue
                    print ("    ---------------- Step 4 ---------------")  
                    print ("Moving back")     
                    #control.moveDist([front_marker[0],0,0,0,0,0])
                    # while control.isOkey() == False :
                        # continue
                    print ("    ---------------- Step 5 ---------------")
                    print ("Moving To Front Of Marker")     
                    #control.moveDist([front_marker])
                    # while control.isOkey() == False :
                        # continue              
                    print ("    ---------------- Step 6 ---------------")
                    print ("Turning Back")
                    #control.moveDist([0,0,0,0,0,180])
                last_move = False
                finish_marker = True
        elif finish_marker and last_move :
            print ("*-*-*-*-*-* FINISH QUALIFICATION *-*-*-*-*-*")
            while True :
                #control.stop()
                a = 1 
        last = res.data.type               
        rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node('mission_qualify',anonymous=False)
    main()
