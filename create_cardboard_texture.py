import os, sys
import numpy as np
import matplotlib.pyplot as plt
from skimage.transform import resize, rescale


ppi = 55
word_ppi = 100
ppc = ppi / 2.54
word_ppc = word_ppi / 2.54

base_path = r'c:\wut\ratsplaydoom\cardboard\doom-ii-map01-entryway-3dmodel\textures'  # TODO to linux style
to_path = r'c:\wut\ratsplaydoom\cardboard\cardboard_textures'

room_name = 'oszlopok2'
wall_width = 3
wall_height = 8  # 10. and 13.
to_size_cm = (wall_height, wall_width)
src_img_name = 'modwall3.png'
stretch = True
fix_to_height = False
just_tile = False
tile_size_cm = (13/3, 3/2)
height_cut_start = 0  # def: 0
height_cut_end = 64  # def: big enough
width_cut_start = 0
width_cut_end = 9999
# bot_cut = 0

src_img_path = os.path.join(base_path, src_img_name)
src_img = plt.imread(src_img_path)
src_img = src_img[height_cut_start:height_cut_end, width_cut_start:width_cut_end]

from_size_cm = (src_img.shape[0] / ppc, src_img.shape[1] / ppc)
os.makedirs(fr'{to_path}\{room_name}', exist_ok=True)  # TODO to linux style

print(f'img pix: {src_img.shape} cm: {from_size_cm[0]:.2f} x {from_size_cm[1]:.2f}')

to_size_pix = (int(to_size_cm[0] * ppc), int(to_size_cm[1] * ppc))
tile_size_px = (tile_size_cm[0] * ppc, tile_size_cm[1] * ppc)

if fix_to_height:
    size_multiplier = to_size_pix[0] / src_img.shape[0]
    tile_size_px = (to_size_pix[0], int(src_img.shape[1] * size_multiplier))

if stretch:
    new_img = resize(src_img, to_size_pix, order=0)
elif just_tile or fix_to_height:
    new_img = resize(src_img, tile_size_px, order=0)
    # tile img if not enough width/height
    height_tile = int(np.ceil(to_size_pix[0] / new_img.shape[0]))
    width_tile = int(np.ceil(to_size_pix[1] / new_img.shape[1]))
    new_img = np.tile(new_img, (height_tile, width_tile, 1))
    # new_img = np.concatenate([new_img] * width_tile, axis=1)
    # new_img = np.concatenate([new_img] * width_tile, axis=0)
    new_img = new_img[:to_size_pix[0], :to_size_pix[1], :]
else:
    # resize side that's longer than original
    incr_size_to = (to_size_pix[0] if to_size_pix[0] > src_img.shape[0] else src_img.shape[0],
                    to_size_pix[1] if to_size_pix[1] > src_img.shape[1] else src_img.shape[1])

    reps = (int(np.ceil(to_size_pix[0] / src_img.shape[0])), int(np.ceil(to_size_pix[1] / src_img.shape[1])), 1)
    new_img = np.tile(src_img, reps)
    new_img = new_img[:to_size_pix[0], :to_size_pix[1]]

# resize to word ppi
resize_by = word_ppi / ppi
new_img = rescale(new_img, (resize_by, resize_by, 1), order=0)

print(f'resized img pix: {new_img.shape} cm: {new_img.shape[0] / word_ppc:.2f} x {new_img.shape[1] / word_ppc:.2f}')

new_img_size_cm = (new_img.shape[0] / word_ppc, new_img.shape[1] / word_ppc)
print(f'final img pix: {new_img.shape} cm: {new_img_size_cm[0]:.2f} x {new_img_size_cm[1]:.2f}')

stretch_str = '_stretch' if stretch else ''
fix_to_height_str = '_fixh' if fix_to_height else ''
new_img_name = f'{src_img_name[:src_img_name.find(".")]}_{new_img_size_cm[0]:.1f}-{new_img_size_cm[1]:.1f}{stretch_str}{fix_to_height_str}.png'
new_img_dir = os.path.join(to_path, room_name)
new_img_path = os.path.join(new_img_dir, new_img_name)
os.makedirs(new_img_dir, exist_ok=True)

plt.imsave(new_img_path, new_img)

if abs(new_img_size_cm[0] - to_size_cm[0]) > 0.5 or abs(new_img_size_cm[1] - to_size_cm[1]) > 0.5:
    print('GENERATED IMG IS OFF!', file=sys.stderr)

print('final img saved to:', new_img_path)
