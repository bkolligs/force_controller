#!/usr/bin/env python
# note this is a relative import. Make sure your IDE is configured to allow this
import simulation as s
import numpy as np
import matplotlib.pyplot as plt
import argparse

def movement_sequence(step):
    output = np.zeros(6)

    if step < 1000:
        output = np.array([0.2, 0, -1, 0, 0, -1])

    if step >= 1000 and step < 1500:
        output = np.array([0.2, 0.1, -1.1, 0, 0.2, -1])

    if step >= 1500 and step < 2000:
        output = np.array([0.5, -0.1, -0.5, 0, 0, 0])

    
    return output



def pd_controller(plant):
    '''
    Closed loop simulation for  moving to a desired position

    Args:
        sim - system class object

    Returns:
        (ndarray) - (STEPS x SENSORS) [j1 input, j2 input, j1 pos, j2 pos, j1 vel, j2 vel]
                    STEPS: sim_length + 1
                    SENSORS: len(sys_in) + len(sys_out)
    '''
    step = plant.display_system()
    # gather system output
    y = plant.system_output('pd')
    j_i = plant._joint_number

    # simple PD controller
    des = movement_sequence(step)
    err = des - y[:j_i]
    
    des_dot = np.array([0, 0, 0, 0, 0, 0])
    err_dot = des_dot - y[j_i:]

    kp = np.diag(np.array([10, 100, 50, 5, 20, 5]))
    kd = np.diag(np.array([1, 10, 10, 0.1, 0.15, 0]))
    sys_in = kp @ err + kd @ err_dot

    u = plant.system_input(sys_in)
    return np.concatenate((u, y))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str,
                        default='xarm', help=' > problem number to run')
    parser.add_argument('--sim_length', type=int,
                        default=5000, help=' > number of simulation steps')
    parser.add_argument('--plot', type=bool,
                        default=True, help=' > plot when the simulation is done')
    parser.add_argument('--plot_torque', type=bool,
                        default=False, help=' > plot when the simulation is done')

    # question number to run

    args = parser.parse_args()
    robot = args.robot
    sim_length = args.sim_length

    # simulate the plant
    funcs = {'xarm': pd_controller}

    plant = s.Plant(robot, funcs)
    sim_data = plant.simulate(sim_length)

    # this graph is for closed loop position control
    if args.plot:
        joi = 5
        plt.plot(sim_data[:, (joi - 1) + 6], 'r', label="Joint {0} Output Position".format(joi))
        if args.plot_torque:
            plt.plot(sim_data[:, joi - 1], 'r--', label="Joint {0} Input Torque".format(joi))
        plt.title("Control Input and Output")
        plt.legend()
    plt.show()
