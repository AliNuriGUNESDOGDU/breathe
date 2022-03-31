#!/usr/bin/env python
import datetime
import pexpect
import os
import rosparam
import rospy
import timeit

class BLECTL(object):
    """This class implements fsr read class
    send_default():
    method3():
    """
    def __init__(self):
        self.f = None
        self.do_once = False
        self.child = pexpect.spawn("gatttool -t random -b CC:4A:71:9E:EE:15 -I")
        self.child.sendline("connect")
        self.child.expect("Connection successful", timeout=5)        
        # Upload parameter server
        file_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config/ble_param.yaml')
        paramlist = rosparam.load_file(file_path,"ble_params")
        for params, ns in paramlist:
            rosparam.upload_params(ns, params)

    def cyclic_func(self):
        rate = 5 # Hz
        sample = rospy.Rate(rate)
        while not rospy.is_shutdown():
            start = timeit.default_timer()
            self.ble_step()
            stop = timeit.default_timer()
            ble_time = stop-start
            print(ble_time)
            sample.sleep()
            pass

    def ble_step(self):
        write_enable = rosparam.get_param("ble_params/write2file")
        if write_enable:
            if self.do_once:
                self.f = open(rosparam.get_param("ble_params/name_file")+".txt", "a")
                self.do_once = False                
            self.child.sendline("char-read-uuid 2a37")
            self.child.expect("handle: 0x0029 	 value: ", timeout=10)
            self.child.expect("\r\n", timeout=10)
            print(str(self.child.before))
            self.f.write(str(datetime.datetime.now())+"\t")
            self.f.write(self.child.before + "\n")
        else:
            self.do_once = True
            if (self.f != None):
                self.f.close()
                self.f = None
            
        
        #print(stop-start)

if __name__ == '__main__':
    try:
        rospy.init_node('fsr_test', anonymous=True)
        ble = BLECTL()
        print("BLE Started")
        ble.cyclic_func()

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
