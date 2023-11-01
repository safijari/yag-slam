# This file contains functions which can convert an existing map image to
# a graph which can then be used to continue mapping process inside yag slam

from numba import njit
from skimage.segmentation import slic, mark_boundaries, find_boundaries
from yag_slam.models import LocalizedRangeScan
from yag_slam.raytracing import run_raytracing_sweep
from tqdm import tqdm
import numpy as np
from collections import defaultdict
import cv2

def pixel_to_meters(resolution, origin, h, x, y):
    return (x*resolution) + origin[0], ((y)*resolution) + origin[1]

def segment_map(imin, verbose=False): 
    imin = imin.copy()

    imin[imin < 254] = 0

    test_image = imin
    s = 11
    test_image = 255-test_image
    test_image = cv2.dilate(test_image, np.ones((s, s)))
    test_image = cv2.erode(test_image, np.ones((s, s)))
    test_image = 255-test_image

    imin = test_image

    numSegments = int(imin.sum() // 600000) * 2
    print("creating {} segments".format(numSegments))
    segments = slic(imin, n_segments = numSegments, sigma = 0, compactness=0.01, mask=imin, channel_axis=None)
    if verbose:
        import matplotlib.pyplot as plt
        plt.imshow((mark_boundaries(imin, segments, color=(1, 0, 0))*255).astype('uint8'))
    return segments

def determine_centroids(segments):
    xx = []
    yy = []
    centroid_map = {}
    print("Finding centroids")
    for sid in tqdm(np.unique(segments)[1:]):
        yvals, xvals = np.where(segments == sid)
        centroid_map[sid-1] = (np.mean(xvals), np.mean(yvals))
    return centroid_map

def create_edges(segments):
    edge_map = defaultdict(int)
    for y, x in zip(*np.where(find_boundaries(segments) == True)):
        uniques = [i-1 for i in sorted(np.unique(segments[y-2:y+2, x-2:x+2])) if i]
        if len(uniques) == 2:
            key = f"{uniques[0]}_{uniques[1]}"
            edge_map[key] += 1
    edges = []
    for key, freq in edge_map.items():
        if freq > 3:
            one, two = [int(s) for s in key.split("_")]
            edges.append((one, two))

    return edges

def map_to_graph(map_image, resolution, origin):
    im = map_image
    segments = segment_map(map_image, verbose=False)
    centroid_map = determine_centroids(segments)
    edges =  create_edges(segments)
    angles = np.arange(-180, 180, 0.25)[:-1]
    # import ipdb; ipdb.set_trace()
    lrss = []
    rtim = im.copy()
    for cm in tqdm(range(0, len(centroid_map))):
        ranges = []
        x = centroid_map[cm][0]
        y = centroid_map[cm][1]
        for angle, info in zip(angles, run_raytracing_sweep(rtim, angles, x, y)):
            rng = info.length*resolution
            if rng > 20:
                rng = 100
            ranges.append(rng)
#         print(x, y)
        x, y = pixel_to_meters(resolution, origin, im.shape[0], x, y)
#         print(x, y)
        lrs = LocalizedRangeScan(ranges, -np.pi, np.pi - np.deg2rad(0.25), np.deg2rad(0.25), 0, 30, 20, 
                                x, y, 0)
        lrs.num = cm
        lrss.append(lrs)

    return lrss, edges