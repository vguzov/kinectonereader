#! /usr/bin/env python

import numpy as np
from skimage.io import imsave
from os.path import isfile
from PIL import Image, ImageDraw
from sklearn.cluster import DBSCAN
import sys
import os

zthresh = (1000, 6000)
KINECT_SHAPE = (424, 512)
zmin, zmax = zthresh
steady_points = [1, 2, 3, 5, 9, 14, 18]


def background_remove_oldest(vmap, background):
    vmap[np.logical_and(background > 0.01, np.abs(vmap - background) < 0.03)] = -1
    return vmap


def background_remove(vmap, points, ifr, debug_output = False):
    h, w = vmap.shape
    features = np.vstack(((np.arange(0, h)[:, np.newaxis] * np.ones((1, w))).flatten(),
                          (np.arange(0, w) * np.ones((h, 1))).flatten(), vmap.flatten() * 100)).transpose()
    scan = DBSCAN(eps=5, min_samples=50, metric='euclidean', algorithm='auto')
    labels = scan.fit_predict(features)
    labels = labels.reshape((h, w))
    pts = [(points[i, 1], points[i, 0]) for i in range(len(points))]
    mask = np.zeros((h, w), dtype=np.bool)
    for i in steady_points:
        if (pts[i][0] < h) and (pts[i][1] < w) and labels[int(pts[i][0]), int(pts[i][1])] != -1:
            mask = np.logical_or(mask, labels == labels[int(pts[i][0]), int(pts[i][1])])
    ans = np.where(mask, vmap, np.ones((h, w))*(-1))
    if debug_output:
        print(np.unique(labels))
        colors = {}
        pic = np.zeros((h, w, 3))
        for i in range(h):
            for j in range(w):
                if not (labels[i, j] in colors):
                    colors[labels[i, j]] = (np.random.random(), np.random.random(), np.random.random())
                if labels[i, j] > -1:
                    pic[i, j, :] = colors[labels[i, j]]
        imsave("pic" + str(ifr) + ".png", pic)
    return ans


def load_npy(path):
    arr = np.load(path).astype(np.float32)
    arr[arr > zthresh[1]] = zthresh[1]
    arr[arr < zthresh[0]] = zthresh[0]
    return arr


def mirror(vmap,points=None):
    if not (points is None):
        points[:,0] = vmap.shape[1] - points[:, 0] - 1
    return (vmap[:,::-1],points)


skel_lines = [
    # Torso
    (3, 2),
    (2, 4),
    (2, 8),
    (2, 1),
    (1, 0),
    (0, 12),
    (0, 16),
    # Left arm
    (4, 5),
    (5, 6),
    (6, 7),
    # Right arm
    (8, 9),
    (9, 10),
    (10, 11),
    # Left leg
    (12, 13),
    (13, 14),
    (14, 15),
    # Right leg
    (16, 17),
    (17, 18),
    (18, 19)
]


def draw_n_save(filename, img, points = None):
    img = img.copy()
    img[img<0]=0
    if points is None:
        imsave(filename, img)
        return
    radius = 0.008 * KINECT_SHAPE[0]
    point_color = (255, 0, 0)
    line_color = (0, 255, 0)
    line_width = 1
    img = (img * 255).astype(np.uint8).copy()
    pil_img = Image.fromarray(img).convert(mode="RGB")
    pil_draw = ImageDraw.Draw(pil_img)
    for line in skel_lines:
        coords = np.hstack((points[line[0], :], points[line[1], :]))
        if not np.isnan(coords).any():
            pil_draw.line((coords[0], coords[1], coords[2], coords[3]), fill=line_color, width=line_width)
    for j in range(points.shape[0]):
        pt = points[j, :]
        if not np.isnan(pt).any():
            pil_draw.ellipse((pt[0] - radius, pt[1] - radius, pt[0] + radius, pt[1] + radius), fill=point_color if j in steady_points else (0,0,255))
    pil_img.save(filename)


def txtconvert(text):
    if text == b'-1.#J':
        return np.nan
    else:
        return float(text)


if len(sys.argv) < 3:
    print("Usage: %s input_folder output_folder [frames] [-s start_frame] [-v] [-clean]" % (sys.argv[0]))
    sys.exit(1)
arguments = {'visualise':False, 'start_frame': 0, 'frames':100, 'only_clean':False}
arguments['input_folder'] = sys.argv[1]
arguments['output_folder'] = sys.argv[2]
arg_i=3
while arg_i < len(sys.argv):
    arg = sys.argv[arg_i]
    arg_i+=1
    if arg == '-s':
        if len(sys.argv)==arg_i:
            print("Parameter error")
            sys.exit(0)
        arguments['start_frame'] = int(sys.argv[arg_i])
        arg_i+=1
    elif arg == '-v':
        arguments['visualise'] = True
    elif arg == '-clean':
        arguments['only_clean'] = True
    else:
        arguments['frames'] = int(sys.argv[3])
if not os.path.exists(arguments['output_folder']):
    os.makedirs(arguments['output_folder'])

skipped_frames=0
print("Min:", zmin, "max:", zmax)
print("Creating pictures")
vmap = load_npy(os.path.join(arguments['input_folder'], "background.npy"))
background = (vmap - zmin) / (zmax - zmin)
background = background.copy()
imsave(os.path.join(arguments['output_folder'], 'background.png'), mirror(background)[0])
for i in range(1, arguments['frames'] + 1):
    vmap = load_npy(os.path.join(arguments['input_folder'], "output" + str(i + arguments['start_frame']) + ".npy"))
    vmap = (vmap - zmin) / (zmax - zmin)
    vmap = background_remove_oldest(vmap, background)
    if isfile(os.path.join(arguments['input_folder'], "output" + str(i + arguments['start_frame']) + ".txt")):
        points = np.loadtxt(os.path.join(arguments['input_folder'], "output" + str(i + arguments['start_frame']) + ".txt"),
                            usecols=(2, 3), comments=';', converters={2: txtconvert, 3: txtconvert})
        vmap = background_remove(vmap, points, i)
    else:
        points = None
    vmap,points = mirror(vmap,points)
    if not (arguments['only_clean'] and (points is None)):
        if (arguments['visualise']):
            draw_n_save(os.path.join(arguments['output_folder'], str(i + arguments['start_frame']) + '.png'), vmap, points)
        np.save(os.path.join(arguments['output_folder'], str(i + arguments['start_frame']) + '.npy'),vmap.astype(np.float64))
    else:
        skipped_frames+=1
    if skipped_frames>0:
        print('\rProcessed %d frames, %d skipped' % (i,skipped_frames),end='', flush=True)
    else:
        print('\rProcessed %d frames' % (i), end='', flush=True)
