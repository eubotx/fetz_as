import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

class ArenaSelector:
    def __init__(self, img):
        self.img = img
        self.fig, self.axes = plt.subplots(figsize=(8, 8))
        self.axes.set_title('click on points')
        self.axes.imshow(img)
        self.arena_selection = np.empty((0, 2))

        self.fig.canvas.mpl_connect('button_press_event', self)
        plt.show()

    def __call__(self, event):
        if event.xdata is not None and event.ydata is not None:
            print('click', event)
            self.arena_selection = np.append(self.arena_selection, np.array([[event.xdata, event.ydata]]), axis=0)
            print(self.arena_selection.shape)
            print(self.arena_selection)

            self.axes.cla()
            self.axes.imshow(self.img)
            self.axes.plot(self.arena_selection[:, 0], self.arena_selection[:, 1], 'o')
            hull = ConvexHull(self.arena_selection)
            for simplex in hull.simplices:
                self.axes.plot(self.arena_selection[simplex, 0], self.arena_selection[simplex, 1], 'r-')

            self.fig.canvas.draw()