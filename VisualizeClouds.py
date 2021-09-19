import io
import cv2
import matplotlib.pyplot as plt
from PIL import Image
import os
import numpy as np
from tqdm import tqdm

visualize_dir = r"D:\Project\Mybullet\Visualize"
GIF_PATH = 'force_y.gif'


def visualize_cloud(clouds, save_path):
    imgs = []
    print("converting gif...")
    for cloud in tqdm(clouds):
        img = cloud2scatter(cloud)
        img = Image.fromarray(img)
        imgs.append(img)

    imgs[0].save(os.path.join(visualize_dir, GIF_PATH), save_all=True,
                 append_images=imgs[1:], duration=40, loop=0)



def cloud2scatter(cloud):
    fig = plt.figure(figsize=(7, 7))
    X, Y, Z = zip(*cloud)
    plt.scatter(X, Y, s=5)
    plt.xlim([0.5, 2.5])
    plt.ylim([-2.5, -0.5])
    buf = io.BytesIO()
    fig.savefig(buf, format='png', dpi=180)
    buf.seek(0)
    img_arr = np.frombuffer(buf.getvalue(), dtype=np.uint8)
    buf.close()
    img = cv2.imdecode(img_arr, 1)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    plt.close()

    return img

