#!/usr/bin/env python
from copy import deepcopy
from ctypes import sizeof
import os
import rospy
import matplotlib.pyplot as plt
import numpy
class SubjectData(object):
    def __init__(self):
        self.experiment = []
        self.name = "subject"
        pass

class Experiment(object):
    def __init__(self):
        self.bpm = 0.0
        self.amp = 0.0
        self.task = 0
        pass

class TEXTCTL(object):
    """This class implements fsr read class
    send_default():
    method3():
    """
    def __init__(self):
        self.f = None
        self.do_once = False  
        self.subjects = {}
        # Upload parameter server
        self.file_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'experiments')
        self.file_list = os.listdir(self.file_path)
        os.chdir(self.file_path)
        for file in self.file_list:
            f = open(file, "r")
            file_head = f.readline()
            if  file_head != 'Test File!\n':
                lines = f.readlines()
                accelerations = []
                quaternians = []
                hex_str_list_down = ["","" ]
                for line in lines:
                    if len(line) == 73:
                        new_line = line[27:71]
                        hex_str_list_down[0] = new_line[0:20]
                        hex_str_list_down[1] = new_line[23:44]
                        for hex_str_mem in hex_str_list_down:                            
                            hex_str_list = hex_str_mem.split()
                            hex_int_list = [(int(hex_str,16)-127.0)/128.0 for hex_str in hex_str_list]
                            
                            accelerations.append(deepcopy(hex_int_list[0:3]))
                            quaternians.append(deepcopy(hex_int_list[3:7]))
                accelerations = numpy.array(accelerations)
                quaternians = numpy.array(quaternians)
                '''
                plt.plot(numpy.cumsum(accelerations[:,0]-numpy.mean(accelerations[:,0])))
                plt.plot(numpy.cumsum(accelerations[:,1]-numpy.mean(accelerations[:,1])))
                plt.plot(numpy.cumsum(accelerations[:,2]-numpy.mean(accelerations[:,2])))
                '''
                mean_accel = [0 , 0, 0]
                std_accel = [0 , 0, 0]
                abs_mean_accel = [0 , 0, 0]
                abs_peak_accel = [0 , 0, 0]
                for i in range(0,3):
                    mean_accel[i] = numpy.mean(accelerations[:,i])
                    std_accel[i] = numpy.std(accelerations[:,i])
                    abs_mean_accel[i] = numpy.mean(numpy.abs(accelerations[:,i]))
                    abs_peak_accel[i] = numpy.max(numpy.abs(accelerations[:,i]))
                print(file)
                print(mean_accel)
                print(std_accel)
                print(abs_peak_accel)
                print(abs_mean_accel)

                name_spec = file.split("amp",1)
                subject_name = name_spec[0][0:-1]
                part_number = name_spec[0][-1]
                amp_bpm = name_spec[1].split("bpm",1)
                amp_exp = amp_bpm[0]
                bpm_task = amp_bpm[1].split("task",1)
                bpm_exp = bpm_task[0]
                task_exp = bpm_task[1][0]
                if self.subjects.get(subject_name) == None:
                    self.subjects.update({subject_name : [(part_number,amp_exp,bpm_exp,task_exp,accelerations)]})
                else:
                    subject_data = self.subjects.get(subject_name)
                    subject_data.append((part_number,amp_exp,bpm_exp,task_exp))
                    self.subjects.update({subject_name : subject_data})
        
        print(self.subjects.get("faruk")[0][0])

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
