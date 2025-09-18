import matplotlib.pyplot as plt
from collections import deque
import queue
import numpy as np

fig = None 
ax = None
robot_plot = None
scat = None

def initialisation_Affichage(x_robot,y_robot):
    global fig, ax, robot_plot, scat
    fig, ax = plt.subplots()
    scat = ax.scatter([], [], s=5, c='blue')   # points du lidar
    robot_plot = ax.scatter([x_robot], [y_robot], s=50, c='red', marker='x')  # robot en rouge
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2000)
    ax.set_aspect('equal')

def affichage(pile_points,x_robot,y_robot):
    global fig, ax, robot_plot, scat
    buffer_points = deque(maxlen=2000)
    while True:
        try:
            try:
                p = pile_points.get(timeout=0.1)
                buffer_points.append(p)
            except queue.Empty:
                pass

            while not pile_points.empty():
                buffer_points.append(pile_points.get_nowait())

            if buffer_points:
                xs, ys = zip(*buffer_points)
                scat.set_offsets(np.c_[xs, ys])

                # met à jour la position du robot (fixe dans ton cas)
                robot_plot.set_offsets([[x_robot, y_robot]])

                plt.pause(0.01)

        except KeyboardInterrupt:
            break
