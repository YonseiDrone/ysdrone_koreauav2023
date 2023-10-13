import glob, argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

def update(frame):
    ax.clear()

    avg_setpoint = np.mean(setpoint[:frame + 1], axis=0)
    ax.scatter(other_pos_3d[frame][:, 0], other_pos_3d[frame][:, 1], other_pos_3d[frame][:, 2], c='red', marker='o', s=2, label='marker')
    ax.scatter(drone_pos_3d[frame][0], drone_pos_3d[frame][1], drone_pos_3d[frame][2], c='green', marker='o', s=30, label='drone_pos')
    ax.scatter(avg_setpoint[0], avg_setpoint[1], avg_setpoint[2], c='blue', marker='o', s=30, label='avg_setpoint')
    ax.scatter(cross_pos_3d[frame][0], cross_pos_3d[frame][1], cross_pos_3d[frame][2], c='orange', marker='o', s=30, label='cross_pos')
    ax.plot([setpoint[frame][0], cross_pos_3d[frame][0]], [setpoint[frame][1], cross_pos_3d[frame][1]], [setpoint[frame][2], cross_pos_3d[frame][2]], c='b', label='normal_vector')
    ax.scatter(setpoint[frame][0], setpoint[frame][1], setpoint[frame][2], c='purple', marker='o', s=30, label='setpoint')
    ax.set_title(f'{frame}')

    # # 축 범위 고정
    ax.set_xlim(85, 105)  # x 축 범위 설정
    ax.set_ylim(-170, -190)  # y 축 범위 설정
    ax.set_zlim(5, 15)  # z 축 범위 설정

    ax.legend()
    ax.grid(True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="npy visualizer")
    parser.add_argument('--logdir', dest='logdir', type=str, help='logdir')

    args = parser.parse_args()

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    d = args.logdir
    # d = f'0902/'

    l = len(glob.glob(d+'other_pos_3d_*.npy'))

    other_pos_3d = [np.load(d + f'other_pos_3d_{i}.npy') for i in range(l)]
    drone_pos_3d = [np.load(d + f'drone_pos_3d_{i}.npy') for i in range(l)]
    setpoint = [np.load(d + f'setpoint_{i}.npy') for i in range(l)]
    cross_pos_3d = [np.load(d + f'cross_pos_3d_{i}.npy') for i in range(l)]

    ani = FuncAnimation(fig, update, frames=range(l), repeat=True)
    plt.show()