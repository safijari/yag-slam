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
    return (x*resolution) + origin[0], ((h-y)*resolution) + origin[1]

def segment_map(imin, verbose=False, density=1): 
    imin = imin.copy()

    imin[imin < 254] = 0

    test_image = imin
    s = 11
    test_image = 255-test_image
    test_image = cv2.dilate(test_image, np.ones((s, s)))
    test_image = cv2.erode(test_image, np.ones((s, s)))
    test_image = 255-test_image

    imin = test_image

    numSegments = int(imin.sum() // 600000 * density) 
    print("creating {} segments".format(numSegments))
    try:
        segments = slic(imin, n_segments = numSegments, sigma = 0, compactness=0.01, mask=imin, channel_axis=None)
    except Exception:
        segments = slic(imin, n_segments = numSegments, sigma = 0, compactness=0.01, mask=imin)
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

def map_to_graph(map_image, resolution, origin, density=1):
    im = map_image
    segments = segment_map(map_image, verbose=False, density=density)
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
        for angle, info in zip(angles, run_raytracing_sweep(rtim, angles[::-1], x, y)):
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

def map_to_graphslam(slam_fake, map_image, resolution, origin, density=1):
    scans, edges = map_to_graph(map_image, resolution, origin, density)
    scan_map = {s.num: s for s in scans}

    in_edges = set([e[0] for e in edges] + [e[1] for e in edges])

    for ii, scan in enumerate(scans):
        slam_fake.add_vertex(scan)

    for ii, (frm, to) in enumerate(edges):
        slam_fake.link_scans(scan_map[frm], scan_map[to], None, np.identity(3)*10**-12)

    # to get rid of any nodes that did not have edges
    slam_fake.vertices = [v for v in slam_fake.graph.vertices if v.obj.num in in_edges]
    for ii, v in enumerate(slam_fake.graph.vertices):
        v.obj.num = ii

    return slam_fake