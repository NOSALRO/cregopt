import numpy as np
import matplotlib.pyplot as plt


def read_csv(filename):
    return np.loadtxt(filename, delimiter=",")


def plot_2d_mesh(data):
    rows, cols = data.shape
    x = np.arange(cols)
    y = np.arange(rows)
    x, y = np.meshgrid(x, y)
    z = data

    plt.pcolor(x, y, z)
    plt.show()


def main():
    filename = "step.csv"
    data = read_csv(filename)
    plot_2d_mesh(data)


if __name__ == "__main__":
    main()
