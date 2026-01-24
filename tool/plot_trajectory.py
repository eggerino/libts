import matplotlib.pyplot as plt
import pandas as pd


def read_data():
    return pd.read_csv(open(0))


def try_add_plot_pose_trajectory(fig: plt.Figure, data: pd.DataFrame):
    if "x" not in data:
        return

    ax = fig.add_subplot(projection="3d")
    x, y, z = data["x"], data["y"], data["z"]
    ax.plot(x, y, z, label="Pose")
    ax.legend()
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")


def main():
    data = read_data()
    fig = plt.figure()
    try_add_plot_pose_trajectory(fig, data)
    plt.show()


if __name__ == "__main__":
    main()
