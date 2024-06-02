# Copyright 2024 Jariullah Safi

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from numba import njit
from numba.experimental import jitclass
from numba.types import float32
import numpy as np

@jitclass([("x", float32), ("y", float32)])
class Point2(object):
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    @property
    def val(self):
        return (self.x, self.y)

    @property
    def xi(self):
        return int(np.round(self.x))

    @property
    def yi(self):
        return int(np.round(self.y))

    def __sub__(self, other):
        return Point2(self.x - other.x, self.y - other.y)

    def __add__(self, other):
        return Point2(self.x + other.x, self.y + other.y)

    def div(self, val):
        return Point2(self.x / val, self.y / val)

    @property
    def norm(self):
        return np.sqrt(self.x**2 + self.y**2)


@jitclass([("start", Point2.class_type.instance_type), ("end", Point2.class_type.instance_type)])
class RayInfo(object):
    def __init__(self, start, end):
        self.start = start
        self.end = end

    @property
    def length(self):
        return (self.end - self.start).norm

@njit(cache=True)
def trace_ray(img, angle, sx, sy):
    h, w = img.shape[:2]
    angle = np.deg2rad(angle)
    should_stop = False
    spt = Point2(sx, sy)
    pt = Point2(spt.x, spt.y)
    speed = 1
    should_run = True
    while should_run:
        try:
            val = img[pt.yi, pt.xi]
            if val < 210:
                should_run = False
                
            pt = Point2(pt.x + speed * np.cos(angle), pt.y + speed * np.sin(angle))
            if 180 < val < 210 and should_run == False:
                pt = Point2(pt.x + 1000 * np.cos(angle), pt.y + 1000 * np.sin(angle))
        except Exception:
            should_run = False

        if pt.yi < 1 or pt.xi < 1 or pt.xi >= w - 1 or pt.yi >= h - 1:
            should_run = False

    return RayInfo(spt, pt)


@njit(cache=True)
def run_raytracing_sweep(img, angles, sx, sy):
    return [trace_ray(img, angle, sx, sy) for angle in angles]