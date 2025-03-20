import numpy as np
import matplotlib.pyplot as plt
from PIL import Image, ImageFilter

filepath = "/Users/ttorres/Documents/robofetz/test_data/live-2024-05m26s.png"
img_pil = Image.open(filepath)
img = np.asarray(Image.open(filepath))

### Img in HSV
img_hsv = img_pil.convert('HSV')
a = np.asarray(img_hsv)

########
fig, ax = plt.subplots()
ax.set_title('click on points')

imgplot = ax.imshow(a[:,:,2])

def process_button(event):
    print("Button:", event.x, event.y, event.xdata, event.ydata,
                    event.button)

fig.canvas.mpl_connect('button_press_event', process_button)

plt.figure()
plt.plot(img_pil.histogram())

plt.show()

print("Done")
