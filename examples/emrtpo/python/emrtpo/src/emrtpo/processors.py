# -*- coding: utf-8 -*-

import numpy as np
from insitu import VisitInstrumentation, VISIT_CSG_INNER, VISIT_CSG_ELLIPSOID_PRRR, VISIT_CSG_SPHERE_PR, VISIT_VARCENTERING_NODE, VISIT_VARCENTERING_ZONE, VISIT_VARTYPE_SCALAR, VISIT_VARTYPE_VECTOR, VISIT_MESHTYPE_CSG, VISIT_MESHTYPE_POINT, VISIT_CELL_BEAM, VISIT_CELL_TRI, VISIT_MESHTYPE_UNSTRUCTURED


class MetaProcessor(object):

    def __init__(self, processors):
        self.processors = processors

    def __call__(self, r, e, k):
        for processor in self.processors:
            processor(r, e, k)


class PointsProcessor(object):

    def __init__(self, pattern_file=None):
        self.origin_ids = list()
        self.endpoint_ids = list()
        self.shots = list()
        self.uvs = list()
        self.propagation_lengths = list()
        self.geometric_lengths = list()
        self.points = list()
        self.ninteractions = list()

    def __call__(self, r, e, k):
        for n, p in enumerate(e):
            self.points.append((p[0],p[1],p[2]))
            self.origin_ids.append(r['origin_id'])
            self.endpoint_ids.append(r['endpoint_id'])
            self.shots.append(r['shot'])
            self.uvs.append((r['origin_u'], r['origin_v']))
            self.propagation_lengths.append(r['propagation_length'])
            self.geometric_lengths.append(r['geometric_length'])
            self.ninteractions.append(e.shape[0]-1)


class RaysProcessor(object):

    def __init__(self, pattern_file=None):
        self.origin_ids = list()
        self.endpoint_ids = list()
        self.shots = list()
        self.uvs = list()
        self.propagation_lengths = list()
        self.geometric_lengths = list()
        self.points = list()
        self.ninteractions = list()
        self.segments = list()
        self.n_points = 0

    def __call__(self, r, e, k):
        o = r['origin'][0]
        self.points.append((o[0],o[1],o[2]))
        self.n_points += 1
        for n, p in enumerate(e):
            self.segments.append((VISIT_CELL_BEAM, self.n_points, self.n_points-1))
            self.points.append((p[0],p[1],p[2]))
            self.n_points += 1
            self.origin_ids.append(r['origin_id'])
            self.endpoint_ids.append(r['endpoint_id'])
            self.shots.append(r['shot'])
            self.uvs.append((r['origin_u'], r['origin_v']))
            self.propagation_lengths.append(r['propagation_length'])
            self.geometric_lengths.append(r['geometric_length'])
            self.ninteractions.append(e.shape[0]-1)
