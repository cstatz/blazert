#!/usr/local/bin/python

from __future__ import division

import json as js
import numpy as np
import glob
import trimesh
from scipy import interpolate
from emrtpo.raypaths import *
from emrtpo.processors import *

from time import sleep
from insitu import VisitInstrumentation, VISIT_CSG_INNER, VISIT_CSG_ELLIPSOID_PRRR, VISIT_CSG_SPHERE_PR, VISIT_VARCENTERING_NODE, VISIT_VARCENTERING_ZONE, VISIT_VARTYPE_SCALAR, VISIT_VARTYPE_VECTOR, \
    VISIT_MESHTYPE_CSG, VISIT_MESHTYPE_POINT, VISIT_CELL_BEAM, VISIT_CELL_TRI, VISIT_MESHTYPE_UNSTRUCTURED


def read_mesh(filename):
    m = trimesh.load_mesh(filename)

    x = np.ascontiguousarray(m.vertices[:].flatten(), dtype=np.float64)

    # Reorder is neccessary for VisIt: 
    tmp = list()

    for face in m.faces:
        tmp.append(VISIT_CELL_TRI)
        tmp.append(face[0])
        tmp.append(face[1])
        tmp.append(face[2])

    return x, np.ascontiguousarray(tmp, dtype=np.int32), int(len(tmp) // 4)


def read_simulation_metadata(filename):
    files = sorted(glob.glob(filename))

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
    tris[:, 1:] = tmp_tris[:, :]

    ids = np.load(md['ids_file'])
    above_layer = np.asarray(ids[:, 0], dtype=np.float64)
    below_layer = np.asarray(ids[:, 1], dtype=np.float64)
    above = np.asarray(ids[:, 2], dtype=np.float64)
    below = np.asarray(ids[:, 3], dtype=np.float64)
    materials = np.load(md['materials_file'])
    source_range = md['origin_sphere_range']

    return vertices, tris, above_layer, below_layer, above, below, materials, source_range


def c2p(x):
    theta = np.arccos(x[2] / (np.sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)))
    phi = np.arctan2(x[0], x[1])
    if phi < 0.:
        phi += 2 * np.pi

    # phi[phi<0.] = phi[phi<0.]+360.
    return 180 * phi / np.pi, 180 * theta / np.pi


class Pattern(object):

    def __init__(self, pattern_file):
        d = np.loadtxt(pattern_file, comments='#', skiprows=8)

        f = 1
        s = 181 * 361
        #Er = d[f * s:(f + 1) * s, 3] + 1.j * d[f * s:(f + 1) * s, 4]
        Et = d[f * s:(f + 1) * s, 5] + 1.j * d[f * s:(f + 1) * s, 6]
        Ep = d[f * s:(f + 1) * s, 7] + 1.j * d[f * s:(f + 1) * s, 8]
        G = np.sqrt(Et * Et.conj() + Ep * Ep.conj())
        G = G.reshape((361, 181))
        G[:, 0] = np.mean(G[:, 0])
        G[:, -1] = np.mean(G[:, -1])
        x = np.linspace(0, 360, 361)
        y = np.linspace(180, 0, 181)
        #xx, yy = np.meshgrid(x, y)
        self.re = interpolate.interp2d(x, y, np.real(G.T), kind='cubic')
        self.im = interpolate.interp2d(x, y, np.imag(G.T), kind='cubic')


class EMRTPOVisIt(object):

    def __init__(self, emrtpo_input_filename, pattern_filename=None):

        self.md = read_simulation_metadata(emrtpo_input_filename)

        self.pattern = None

        # TODO: Get Pattern filename from emrtpo_input_file
        if pattern_filename:
            self.pattern = Pattern(pattern_filename)

        self.position_idx = 0
        self.input_data = read_simulation_data(self.md[self.position_idx])

        self.ray_points_data = PointsProcessor()
        self.rays_data = RaysProcessor()
        meta = MetaProcessor([self.ray_points_data, self.rays_data])

        self.result_data = read_and_process_binary_emrt_raypaths(self.md[self.position_idx]['results_file'], processor=meta)

        self.propagation_time = 1e-8
        self.position_changed = True

        self.up = np.asarray([0., 0., 1.])
        self.ray_points_gain_data = list()

        for n, o in enumerate(self.ray_points_data.origin_ids):
            u = self.ray_points_data.uvs[n][0][0]
            v = self.ray_points_data.uvs[n][1][0]
            t = self.input_data[1][o[0]]
            test = (1. - u - v) * self.input_data[0][t[1], :] + u * self.input_data[0][t[2], :] + v * self.input_data[0][t[3], :]
            test /= np.linalg.norm(test)

            # TODO: Rotate the pattern
            if self.pattern:
                self.ray_points_gain_data.append(self.pattern.re(*c2p(test)))
            else:
                self.ray_points_gain_data.append(1.)

    def above_layer(self, *args, **kwargs):
        return self.input_data[2]

    def below_layer(self, *args, **kwargs):
        return self.input_data[3]

    def above(self, *args, **kwargs):
        return self.input_data[4]

    def below(self, *args, **kwargs):
        return self.input_data[5]

    def ray_points_gain(self, *args, **kwargs):
        return np.ascontiguousarray(np.asarray(self.ray_points_gain_data), dtype=np.float64)

    def ray_points_mesh(self, *args, **kwargs):
        p = np.ascontiguousarray(np.asarray(self.ray_points_data.points), dtype=np.float64)
        return p, None, None, 3

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

    def rays_shot(self, *args, **kwargs):
        p = np.ascontiguousarray(self.rays_data.shots, dtype=np.float64)
        return p

    def input_mesh(self, *args, **kwargs):
        return np.ascontiguousarray(self.input_data[0].flatten(), dtype=np.float64), np.ascontiguousarray(self.input_data[1].flatten(), dtype=np.int32), int(self.input_data[1].shape[0])

    def rays_mesh(self, *args, **kwargs):
        return np.ascontiguousarray(np.asarray(self.rays_data.points).flatten(), dtype=np.float64), np.ascontiguousarray(np.asarray(self.rays_data.segments).flatten(), dtype=np.int32), int(
            len(self.rays_data.segments))

    def cycle_time_provider(self, *args, **kwargs):
        return self.position_idx, float(self.position_idx)

    def step(self, *args, **kwargs):

        if len(args) > 0:
            if self.position_idx != int(args[0]):
                self.position_idx = int(args[0])
                self.position_changed = True
        sleep(0.05)

    def prop_time(self, *args, **kwargs):
        self.propagation_time = float(args[0])
        self.step()

    def register(self, instrumentation):
        instrumentation.register_generic_command('proptime', self.prop_time, None)
        instrumentation.register_mesh('ray_points_mesh', self.ray_points_mesh, VISIT_MESHTYPE_POINT, 3, number_of_domains=1, domain_title="Domains", domain_piece_name="domain", num_of_groups=0,
                                      XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z")
        instrumentation.register_mesh('input_mesh', self.input_mesh, VISIT_MESHTYPE_UNSTRUCTURED, 3, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z", topological_dimension=3)
        instrumentation.register_mesh('rays_mesh', self.rays_mesh, VISIT_MESHTYPE_UNSTRUCTURED, 3, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z", topological_dimension=3)
        instrumentation.register_variable('above_layer', 'input_mesh', self.above_layer, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('below_layer', 'input_mesh', self.below_layer, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('above_material', 'input_mesh', self.above, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('below_material', 'input_mesh', self.below, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('ray_points_origin_ids', 'ray_points_mesh', self.ray_points_origin_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_endpoint_ids', 'ray_points_mesh', self.ray_points_endpoint_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_propagation_lengths', 'ray_points_mesh', self.ray_points_propagation_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_geometric_lengths', 'ray_points_mesh', self.ray_points_geometric_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_ninteractions', 'ray_points_mesh', self.ray_points_ninteractions, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('ray_points_gain', 'ray_points_mesh', self.ray_points_gain, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')
        instrumentation.register_variable('rays_origin_ids', 'rays_mesh', self.rays_origin_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_endpoint_ids', 'rays_mesh', self.rays_endpoint_ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_propagation_lengths', 'rays_mesh', self.rays_propagation_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_geometric_lengths', 'rays_mesh', self.rays_geometric_lengths, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_ninteractions', 'rays_mesh', self.rays_ninteractions, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')
        instrumentation.register_variable('rays_shot', 'rays_mesh', self.rays_shot, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_ZONE, Units='m')


def run_insitu(emrt_input_fname, pattern_fname):
    csd = EMRTPOVisIt(emrt_input_fname, pattern_fname)
    name = 'EMRTPO'
    description = 'EMRTPO Visit Instrumentation'
    v = VisitInstrumentation(name, description, prefix='./', step=csd.step, cycle_time_provider=csd.cycle_time_provider, trace=False)

    csd.register(v)
    v.run()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Visualize EMRT input and results data.")
    parser.add_argument("emrtpo_input_filename", help="EMRTPI input file")
    parser.add_argument("pattern_filename", help="Pattern File.")

    args = parser.parse_args()
    run_insitu(args.emrtpo_input_fname, args.pattern_filename)
