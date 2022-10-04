import os, sys
import numpy as np
import matplotlib.pyplot as plt
from skimage.transform import resize, rescale


ppi = 55
word_ppi = 100
ppc = ppi / 2.54
word_ppc = word_ppi / 2.54

base_path = r'c:\wut\ratsplaydoom\cardboard\doom-ii-map01-entryway-3dmodel\textures'
to_path = r'c:\wut\ratsplaydoom\cardboard\cardboard_textures'

room_name = 'corridor'
wall_height = 10.  # 10. and 13.
to_size_cm = (wall_height, 7.7)  # h, w
src_img_name = 'bigdoor1.png'
stretch = True
top_cut = 24
bot_cut = 0

src_img_path = os.path.join(base_path, src_img_name)
src_img = plt.imread(src_img_path)
src_img = src_img[top_cut:]
from_size_cm = (src_img.shape[0] / ppc, src_img.shape[1] / ppc)

print(f'img pix: {src_img.shape} cm: {from_size_cm[0]:.2f} x {from_size_cm[1]:.2f}')


to_size_pix = (int(to_size_cm[0] * ppc), int(to_size_cm[1] * ppc))

if stretch:
    new_img = resize(src_img, to_size_pix)
else:
    # resize side that's longer than original
    incr_size_to = (to_size_pix[0] if to_size_pix[0] > src_img.shape[0] else src_img.shape[0],
                    to_size_pix[1] if to_size_pix[1] > src_img.shape[1] else src_img.shape[1])

    reps = (int(np.ceil(to_size_pix[0] / src_img.shape[0])), int(np.ceil(to_size_pix[1] / src_img.shape[1])), 1)
    new_img = np.tile(src_img, reps)
    new_img = new_img[:to_size_pix[0], :to_size_pix[1]]

# resize to word ppi
resize_by = word_ppi / ppi
new_img = rescale(new_img, (resize_by, resize_by, 1))

print(f'resized img pix: {new_img.shape} cm: {new_img.shape[0] / word_ppc:.2f} x {new_img.shape[1] / word_ppc:.2f}')

new_img_size_cm = (new_img.shape[0] / word_ppc, new_img.shape[1] / word_ppc)
print(f'final img pix: {new_img.shape} cm: {new_img_size_cm[0]:.2f} x {new_img_size_cm[1]:.2f}')

stretch_str = '_stretch' if stretch else ''
new_img_name = f'{src_img_name[:src_img_name.find(".")]}_{new_img_size_cm[0]:.1f}-{new_img_size_cm[1]:.1f}{stretch_str}.png'
new_img_dir = os.path.join(to_path, room_name)
new_img_path = os.path.join(new_img_dir, new_img_name)
os.makedirs(new_img_dir, exist_ok=True)

plt.imsave(new_img_path, new_img)

if abs(new_img_size_cm[0] - to_size_cm[0]) > 0.5 or abs(new_img_size_cm[1] - to_size_cm[1]) > 0.5:
    print('GENERATED IMG IS OFF!', file=sys.stderr)

print('final img saved to:', new_img_path)
