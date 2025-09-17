fig, ax = plt.subplots()
scat = ax.scatter([], [], s=5, c='blue')   # points du lidar
robot_plot = ax.scatter([x_robot], [y_robot], s=50, c='red', marker='x')  # robot en rouge
ax.set_xlim(0, 3000)
ax.set_ylim(0, 2000)
ax.set_aspect('equal')

def affichage():
    buffer_points = deque(maxlen=2000)
    while True:
        try:
            try:
                p = q.get(timeout=0.1)
                buffer_points.append(p)
            except queue.Empty:
                pass

            while not q.empty():
                buffer_points.append(q.get_nowait())

            if buffer_points:
                xs, ys = zip(*buffer_points)
                scat.set_offsets(np.c_[xs, ys])

                # met à jour la position du robot (fixe dans ton cas)
                robot_plot.set_offsets([[x_robot, y_robot]])

                plt.pause(0.01)

        except KeyboardInterrupt:
            break
