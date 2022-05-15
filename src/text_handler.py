#!/usr/bin/env python
import datetime
import os
import rosparam
import rospy
import timeit

class TEXTCTL(object):
    """This class implements fsr read class
    send_default():
    method3():
    """
    def __init__(self):
        self.f = None
        self.do_once = False    
        # Upload parameter server
        self.file_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'experiments')
        self.file_list = os.listdir(self.file_path)
        os.chdir(self.file_path)
        self.new_file_list = []
        for file in self.file_list:
            f = open(file, "r")
            file_head = f.readline()
            if  file_head != 'Test File!\n':
                lines = f.readlines()
                accelerations = []
                quaternians = []
                for line in lines:
                    if len(line) == 73:
                        new_line = line[27:71]
                        hex_str_list_down = new_line.split(' 09 ')
                        print(hex_str_list_down)
                        for hex_str_mem in hex_str_list_down:                            
                            hex_str_list = hex_str_mem.split()
                            hex_int_list = [(int(hex_str,16)-127.0)/128.0 for hex_str in hex_str_list]
                            accelerations.append(hex_int_list[0:3])
                            quaternians.append(hex_int_list[3:7])
                            print(hex_int_list[3:7])
                        

                        print(line[27:73])

    def cyclic_func(self):
        rate = 5 # Hz
        pass
            
        
        #print(stop-start)

if __name__ == '__main__':
    try:
        text_handle = TEXTCTL()
        print("BLE Started")

    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
