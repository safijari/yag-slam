# Copyright 2019 Jariullah Safi

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


class LinkLabel(object):
    def __init__(self, mean, covariance):
        self.mean = mean
        self.covariance = covariance


class Vertex(object):
    def __init__(self, obj):
        self.obj = obj
        self.edges = []

    def add_edge(self, edge):
        self.edges.append(edge)

    def get_adjacent_vertices(self):
        out = []
        for edge in self.edges:
            if edge.source != self:
                out.append(edge.source)
            if edge.target != self:
                out.append(edge.target)
        return out


class Edge(object):
    def __init__(self, source, target, link_info):    # Vertices
        self.source = source
        self.target = target
        self.info = link_info    # (interface: mean, covar)
        self.source.add_edge(self)
        self.target.add_edge(self)


class Graph(object):
    def __init__(self):
        self.vertices = []
        self.edges = []

    def add_vertex(self, vertex):
        self.vertices.append(vertex)
        # this is where a node is added to SBA?

    def add_edge(self, edge):
        """
        Karto does this in MapperGraph over the current vertex (obj?) as such:
        - Link to previous obj
        - Link to the "running scans array" (link to chain) (karto keeps track of the means of this obj as well as an array of the same covariance for this obj(???))
          - This has a distance threshold and only links to one obj from the "running scans array" which is the closest (so in general it should only link to the previous obj)
          - I don't know how necessary this is honestly
        - Link to "near chains" (that do not contain this new obj)
          - Get "near chains"
        """
        self.edges.append(edge)
        # this is where a constrant is added to SBA?


def do_breadth_first_traversal(start_vert, visit_fn, return_objs=True):
    """
    visit_fn takes in first node and current node and returns
    true if the traversal should continue
    """
    to_visit = []
    seen_verts = set()
    valid_verts = []

    to_visit.append(start_vert)
    seen_verts.add(start_vert)

    while to_visit:
        vert = to_visit.pop()
        if (not visit_fn(start_vert, vert)):
            continue

        valid_verts.append(vert)

        for avert in vert.get_adjacent_vertices():
            if avert in seen_verts:
                continue
            to_visit.append(avert)
            seen_verts.add(avert)

    if return_objs:
        return [v.obj for v in valid_verts]
    return valid_verts
