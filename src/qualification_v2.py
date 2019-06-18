#!/usr/bin/python2
from __future__ import print_function
import rospy
import math
from zeabus_utility.srv import VisionSrvQualification
from zeabus.control.command_interfaces import CommandInterfaces
from std_msgs.msg import String

control = CommandInterfaces('mission_qualify_2')

def main():
    # rospy.sleep(20)
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
    control.reset_state()
    control.absolute_z(-1.2)
    r = rospy.Rate(10)
    while control.check_z(0.15) == False :
        print ("Wait for Z")
        r.sleep()
        continue
    while not rospy.is_shutdown():
        current = control.get_state()
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
                    control.relative_xy(0.2 , 0)
                    while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                        continue
                elif res.data.type.data > 0 :
                    while area <= 0.85 and not rospy.is_shutdown():
                        print ("---------------- Finding Gate --------------")
                        res = call(String('qualification'),String('gate'))
                        area = res.data.area.data
                        print ("area = " + str(area))
                        cx = (res.data.point1.x+res.data.point2.x)/2
                        print ("cx = " + str(cx))
                        if cx <= -0.1 and old_cx - cx <= 0.1:
                            print ("Moving Left")
                            control.relative_xy(0,0.2)
                            while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                        elif cx >= 0.1 and old_cx - cx <= 0.1:
                            print ("Moving Right")
                            control.relative_xy(0,-0.2)
                            while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                        else :
                            print ("Moving Forward")
                            control.relative_xy(0.2,0)
                            while control.relative_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                        old_cx = cx
                    # rospy.sleep(0.5)
                    print ("Last Move")
                    last_move = True
                    control.relative_xy(3,0)
                    while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
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
                    control.relative_xy(0.2 , 0)
                    while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                        continue
                #else :
                #if True :
                elif res.data.type.data > 0 :
                    while area <= 0.2 and not rospy.is_shutdown():
                        print ("---------------- Finding Marker --------------")
                        print ("    ---------------- Step 1 ---------------")
                        res = call(String('qualification'),String('marker'))
                        cx = (res.data.point1.x+res.data.point2.x)/2
                        print ("cx = " + str(cx))
                        if cx <= -0.1 :
                            print ("Moving Left")
                            control.relative_xy(0,-0.2)
                            while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                        elif cx >= 0.1 :
                            print ("Moving Right")
                            control.relative_xy(0,0.2)
                            while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                        else :
                            print ("Moving Forward")
                            control.relative_xy(0.2,0)
                            while control.check_xy(0.15 , 0.15) == False and not rospy.is_shutdown():
                                continue
                            # continue
                    front_marker = control.get_state()
                    while res.data.point2.x != -1 and not rospy.is_shutdown():
                        print ("---------------- Finding Marker --------------")
                        print ("    ---------------- Step 2 ---------------")
                        res = call(String('qualification'),String('marker'))
                        area = res.data.area.data
                        print ("Moving Right")
                        control.relative_xy(0,0.2)
                        while control.check_xy(0.15 , 0.15) == False and not rospy.is_shudown():
                            continue
                    print ("    ---------------- Step 3 ---------------")  
                    print ("Moving Forward")
                    control.relative_xy(3,0)
                    rf_marker = control.get_state()
                    while control.check_xy(0.15,0.15) == False and not rospy.is_shutdown():
                        continue                     
                    print ("    ---------------- Step 4 ---------------")  
                    print ("Moving Left")        
                    control.relative_xy(rf_marker[0],rf_marker[1]-3)
                    while control.check_xy(0.15,0.15) == False and not rospy.is_shutdown():
                        continue      
                    rl_marker = control.gat_state()
                    print ("    ---------------- Step 5 ---------------")  
                    print ("Moving back")     
                    control.absolute_xy(rl_marker[0] , front_marker[1])
                    while control.check_xy(0.15,0.15) == False and not rospy.is_shutdown():
                        continue
                    print ("    ---------------- Step 6 ---------------")
                    print ("Moving To Front Of Marker")  
                    control.absolute_xy(front_marker[0] , front_marker[1])
                    while control.check_xy(0.15,0.15) == False and not rospy.is_shutdown():
                        continue   
                    print ("    ---------------- Step 6 ---------------")
                    print ("Turning Back")
                    control.relative_yaw(math.pi)
                    while control.check_yaw(0.15) == False and not rospy.is_shutdown():
                        continue
                last_move = False
                finish_marker = True
        elif finish_marker and last_move :
            print ("*-*-*-*-*-* FINISH QUALIFICATION *-*-*-*-*-*")
            control.deactivate(["x","y","z","roll","pitch","yaw"])
        last = res.data.type               
        rospy.sleep(0.5)

if __name__ == "__main__":
    rospy.init_node('mission_qualify_2',anonymous=False)
    main()
