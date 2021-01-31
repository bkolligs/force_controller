#!/usr/bin/env python
import mujoco_py as mjp
import numpy as np
import time
import os
import matplotlib.pyplot as plt


class Plant:
    '''
    Plant class which will launch mujoco simulation

    Args:
        question - question number
        control_funcs - dictionary of controller functions
    '''

    def __init__(self, robot, control_funcs):
        # xml file to load the model
        self._robot = robot
        # control functions
        self._control_funcs = control_funcs
        # availiable models
        self._models = {'xarm':"models/xarm6.xml"}

        # initialize simulation data into class
        # load the xml file
        self._model = mjp.load_model_from_path(self._models[robot])
        # handle simulation data
        self._sim = mjp.MjSim(self._model)
        # handle rendering
        self._viewer = mjp.MjViewer(self._sim)
        # number of joints
        self._joint_number = len(self._sim.data.ctrl)
        # simulation step number
        self._step = 0


    def sim_start(self, sim_length=None, control_func=None):
        '''
        Starts a mujoco simulation based off of the specified plant file

        Args:
            sim_length - If specified, will stop the simulation after a certain number of steps
            control_func - this is a function that will provide a controller

        Returns:
            output - whatever output the control function outputs
        '''

        output_data = []

        # steps
        self._step = 0
        start = time.time()
        while True:

            # step simulation
            self._sim.step()
            # render simulation
            self._viewer.render()

            output = control_func(self)
            output_data.append(output)

            self._step += 1

            # to terminate simulation
            if sim_length:
                if self._step > sim_length:
                    break

        total_time = (time.time() - start) / 60
        print("Ran simulation for {0:.3f} minutes".format(total_time))
        output = np.array(output_data)
        return output

    def display_system(self):
        '''
        This function displays the input and output of the plants current model

        '''

        # print readings every 100 steps
        if self._step % 500 == 0:
            # detailing current joints
            j_i = self._joint_number
            joint_str = "["
            for i in range(j_i):
                joint_str += "J{} ".format(i + 1)
            joint_str += "]"

            # print system status
            print("STEP:", self._step)
            print("INPUT")
            print("\tControl Inputs {}".format(joint_str), self._sim.data.ctrl)
            print("OUTPUT")
            print("\tPosition Reading {}".format(joint_str),
                  self._sim.data.sensordata[:j_i])
            print("\tVelocity Reading {}".format(joint_str),
                  self._sim.data.sensordata[j_i:2*j_i])
            # print("\tTorque Reading [x1, y1, z1, x2, y2, z2]", sim.data.sensordata[2*j_i:])
        return self._step

    def simulate(self, sim_length=1000000):
        '''
        Selects the question and calls the simulation with the appropriate control function

        Args:
            sim_length - desired length of simulation in steps
        '''
        return self.sim_start(sim_length=sim_length, control_func=self._control_funcs[self._robot])

    def system_output(self, sys_out):
        '''
        Outputs a particular quantity from the simulation

        Args:
            sys_out - quantity to output
                    'all': all sensor data
                    'pos': position data
                    'vel': velocity data
                    'trq': torque data
                    'pd' : pos and vel


        Returns:
            data (ndarray) - the desired quantities
        '''
        j_i = self._joint_number
        sys_out_type = {'pos': self._sim.data.sensordata[:j_i],
                        'vel': self._sim.data.sensordata[j_i:2*j_i],
                        'trq': self._sim.data.sensordata[2*j_i:],
                        'pd': self._sim.data.sensordata[:2*j_i],
                        'all': self._sim.data.sensordata[:]}
        data = sys_out_type[sys_out]
        return data

    def system_input(self, sys_in):
        '''
        Pass a control input into the system

        Args:
            sys_in - system input. Note, must match the size of sim.data.ctrl

        Returns:
            sys_in (ndarray)

        '''
        target_size = self._sim.data.ctrl.shape
        assert target_size == sys_in.shape, "arg 'sys_in' needs to be of size {0}".format(
            target_size)

        self._sim.data.ctrl[:] = sys_in
        return sys_in
