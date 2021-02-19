#!/usr/local/bin/python

from __future__ import division

import json as js
import numpy as np
import os
import glob
import trimesh
from tqdm import tqdm
from scipy import interpolate
from emrt.raypaths import read_and_process_binary_emrt_raypaths

from time import sleep
from insitu import VisitInstrumentation, VISIT_CSG_INNER, VISIT_CSG_ELLIPSOID_PRRR, VISIT_CSG_SPHERE_PR, VISIT_VARCENTERING_NODE, VISIT_VARCENTERING_ZONE, VISIT_VARTYPE_SCALAR, VISIT_VARTYPE_VECTOR, VISIT_MESHTYPE_CSG, VISIT_MESHTYPE_POINT, VISIT_CELL_BEAM, VISIT_CELL_TRI, VISIT_MESHTYPE_UNSTRUCTURED


def read_mesh(filename):


    m = trimesh.load(filename, force='mesh', skip_materials=True)

    x = np.ascontiguousarray(m.vertices[:].flatten(), dtype=np.float64)
    
    # Reorder is neccessary for VisIt: 
    tmp = list()

    for face in m.faces:
        tmp.append(VISIT_CELL_TRI)
        tmp.append(face[0])
        tmp.append(face[1])
        tmp.append(face[2])

    return x, np.ascontiguousarray(tmp, dtype=np.int32), int(len(tmp)//4)  #vertices.view(dtype=np.float32), faces.view(dtype=np.uint32), bounds.view(dtype=np.float64) 


def read_simulation_metadata(fname):

    files = sorted(glob.glob(fname))

    md = list()

    for fn in files:
        with open(fn, 'r') as f:
            md.append(js.load(f))

    return md


def read_simulation_data(md):

    vertices = np.load(md['points_file']) 
    tmp_tris = np.load(md['tris_file'])

    tris = np.zeros((tmp_tris.shape[0], 4), dtype=np.int32)
    tris[:, 0] = VISIT_CELL_TRI
    tris[:,1:] = tmp_tris[:,:]

    return vertices, tris


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


class CreepVisIt(object):

    def __init__(self, creep_input_fname):

        self.md = read_simulation_metadata(creep_input_fname)
        self.position_idx = 0
        self.input_data = read_simulation_data(self.md[self.position_idx])

        self.ray_points_data = PointsProcessor()
        self.rays_data = RaysProcessor()
        meta = MetaProcessor([self.ray_points_data, self.rays_data])

        self.result_data = read_and_process_binary_emrt_raypaths(self.md[self.position_idx]['results_file'], processor = meta)

        self.propagation_time = 1e-8
        self.position_changed = True

        self.up = np.asarray([0.,0.,1.])

    def ray_points_mesh(self, *args, **kwargs):
        p = np.ascontiguousarray(np.asarray(self.ray_points_data.points), dtype=np.float64)
        return p, None, None, 3 # np.ascontiguousarray(,dtype=np.float64)

    def ray_points_origin_ids(self, *args, **kwargs):
        p = np.ascontiguousarray(self.ray_points_data.origin_ids, dtype=np.float64)
        return p

    def ray_points_endpoint_ids(self, *args, **kwargs):
        p = np.ascontiguousarray(self.ray_points_data.endpoint_ids, dtype=np.float64)
        return p

    def ray_points_propagation_lengths(self, *args, **kwargs):
        p = np.ascontiguousarray(self.ray_points_data.propagation_lengths, dtype=np.float64)
        return p

    def ray_points_geometric_lengths(self, *args, **kwargs):
        p = np.ascontiguousarray(self.ray_points_data.geometric_lengths, dtype=np.float64)
        return p

    def ray_points_ninteractions(self, *args, **kwargs):
        p = np.ascontiguousarray(self.ray_points_data.ninteractions, dtype=np.float64)
        return p

    def rays_origin_ids(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.origin_ids, dtype=np.float64)
        return p

    def rays_endpoint_ids(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.endpoint_ids, dtype=np.float64)
        return p

    def rays_propagation_lengths(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.propagation_lengths, dtype=np.float64)
        return p

    def rays_geometric_lengths(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.geometric_lengths, dtype=np.float64)
        return p

    def rays_ninteractions(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.ninteractions, dtype=np.float64)
        return p

    def input_mesh(self, *args, **kwargs):
        return np.ascontiguousarray(self.input_data[0].flatten(), dtype=np.float64), np.ascontiguousarray(self.input_data[1].flatten(), dtype=np.int32), int(self.input_data[1].shape[0]) 

    def rays_mesh(self, *args, **kwargs):
        return np.ascontiguousarray(np.asarray(self.rays_data.points).flatten(), dtype=np.float64), np.ascontiguousarray(np.asarray(self.rays_data.segments).flatten(), dtype=np.int32), int(len(self.rays_data.segments))

    def cycle_time_provider(self, *args, **kwargs):
        return self.position_idx, float(self.position_idx)

    def step(self, *args, **kwargs):

        if len(args)>0:
            if self.position_idx != int(args[0]):
                self.position_idx = int(args[0])
                self.position_changed = True
        sleep(0.05)

    def prop_time(self, *args, **kwargs):
        self.propagation_time = float(args[0])
        self.step()

    def register(self, instrumentation):
        instrumentation.register_generic_command('proptime', self.prop_time, None)
        instrumentation.register_mesh('ray_points_mesh', self.ray_points_mesh, VISIT_MESHTYPE_POINT, 3, number_of_domains=1, domain_title="Domains", domain_piece_name="domain", num_of_groups=0, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z")
        instrumentation.register_mesh('input_mesh', self.input_mesh, VISIT_MESHTYPE_UNSTRUCTURED, 3, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z", topological_dimension=3)
        instrumentation.register_mesh('rays_mesh', self.rays_mesh, VISIT_MESHTYPE_UNSTRUCTURED, 3, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z", topological_dimension=3)
        instrumentation.register_variable('ray_points_origin_ids', 'ray_points_mesh', self.ray_points_origin_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_endpoint_ids', 'ray_points_mesh', self.ray_points_endpoint_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_propagation_lengths', 'ray_points_mesh', self.ray_points_propagation_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_geometric_lengths', 'ray_points_mesh', self.ray_points_geometric_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_ninteractions', 'ray_points_mesh', self.ray_points_ninteractions, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('rays_origin_ids', 'rays_mesh', self.rays_origin_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_endpoint_ids', 'rays_mesh', self.rays_endpoint_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_propagation_lengths', 'rays_mesh', self.rays_propagation_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_geometric_lengths', 'rays_mesh', self.rays_geometric_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_ninteractions', 'rays_mesh', self.rays_ninteractions, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')


def run_insitu(creep_input_fname):

    csd = CreepVisIt(creep_input_fname)
    name = 'Creep'
    description = 'Creep Visit Instrumentation'
    v = VisitInstrumentation(name, description, prefix='./', step=csd.step, cycle_time_provider=csd.cycle_time_provider, trace=False)

    csd.register(v) 
    v.run()


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description="Visualize Creep input and results data.")
    parser.add_argument("creep_input_fname",  help="Creep input file")

    args = parser.parse_args()
    run_insitu(args.creep_input_fname)
