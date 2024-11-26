#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez, Arturo Gil
@Time: February 2024
"""
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.accelerometer import Accelerometer
from robots.objects import CoppeliaObject
from robots.ouster import Ouster
from robots.simulation import Simulation
from robots.husky import HuskyRobot
import pandas as pd


def simulate():
    df = pd.DataFrame(columns=['Step','Time','Position','Orientation:','Joint positions','Transformation matrix',
                               'Mean wheel torque','Wheel torques','Accelerations','Laser Data'])


    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = HuskyRobot(simulation=simulation)
    robot.start(base_name='/HUSKY')
    # Simulate a LiDAR
    lidar = Ouster(simulation=simulation)
    lidar.start(name='/OS1')
    # A dummy object ath the robot center
    robot_center = CoppeliaObject(simulation=simulation)
    robot_center.start(name='/HuskyCenter')
    # an accelerometer
    accel = Accelerometer(simulation=simulation)
    accel.start(name='/Accelerometer')

    # MOVEMENTSDA
    print('MOVING ROBOT')
    robot.move(v=2.5, w=8.0)
    # Must wait till the speed is reached
    # approx 2 seconds (torques
    simulation.wait()

    #  TORQUES
    # now, obtain the mean torques or torques for each wheel
    # during 50 simulat steps.
    for i in range(50):
        time = simulation.sim.getSimulationTime()
        q = robot.get_joint_positions()
        tau = robot.get_mean_wheel_torques()
        tau_wheel = robot.get_wheel_torques()
        axyz = accel.get_accel_data()
        T = robot_center.get_transform()

        position = robot_center.get_position()
        orientation = robot_center.get_orientation()
 #       pi_global = T*pi_

        print("_______________________")
        print("Step:",i,". Time: ", time)
        print("Position: ", position)
        print("Orientation:", orientation)
        print("Joint positions", q)
        print("Transformation matrix: ", T)
        print("Mean wheel torque: ", tau)
        print("Wheel torques: ",tau_wheel)
        print("Accelerations: ",axyz)
        robot.wait()
        data = lidar.get_laser_data()
        print('Laser Data:', data.shape)

        df.loc[i] = {'Step': i,'Time': time,'Position': position,'Orientation:': orientation,
                     'Joint positions': q, 'Transformation matrix': T, 'Mean wheel torque': tau,
                     'Wheel torques': tau_wheel, 'Accelerations': axyz, 'Laser Data': data.shape
                     }

        path = "exp4"

        df.to_csv(path + "/table.csv", sep=';', index=False, header=True)

        pointcloud_filename = path + "/lidar/pointcloud" + str(i) + ".ply"  # You can also use .pcd or .xyz formats
        lidar.save_pointcloud(pointcloud_filename)

    #for i in range(10):
    #    data = lidar.get_laser_data()
    #    lidar.save_pointcloud('lidar/simulated_pointcloud.pcd')
    #    lidar.draw_pointcloud()

    simulation.stop()


if __name__ == "__main__":
    simulate()