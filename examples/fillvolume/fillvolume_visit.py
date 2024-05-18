#!/usr/local/bin/python

from __future__ import division

import json as js
import numpy as np
import os
import glob
import trimesh
from tqdm import tqdm
from scipy import interpolate

from time import sleep
from insitu import VisitInstrumentation, VISIT_VARCENTERING_NODE, VISIT_VARCENTERING_ZONE, VISIT_VARTYPE_SCALAR, VISIT_VARTYPE_VECTOR, VISIT_MESHTYPE_RECTILINEAR, VISIT_MESHTYPE_POINT, VISIT_CELL_BEAM, VISIT_CELL_TRI, VISIT_MESHTYPE_UNSTRUCTURED


class VOLVisIt(object):

    def __init__(self, input_fname):

        self.position_changed = False
        self.position_idx = 0

        tmp = js.load(open(input_fname, 'r'))

        self.x = tmp['x_axis']
        self.y = tmp['y_axis']
        self.z = tmp['z_axis']
        self.vol = np.load(tmp['results_file'])

    def mesh(self, *args, **kwargs):
        return self.x, self.y, self.z

    def ids(self, *args, **kwargs):
        return self.vol

    def cycle_time_provider(self, *args, **kwargs):
        return self.position_idx, float(self.position_idx)

    def step(self, *args, **kwargs):
        sleep(0.05)
        
    def register(self, instrumentation):
        instrumentation.register_mesh('mesh', self.mesh, VISIT_MESHTYPE_RECTILINEAR, 3, XUnits="m", YUnits="m", ZUnits="m", XLabel="x", YLabel="y", ZLabel="z")
        instrumentation.register_variable('ids', 'mesh', self.ids, VISIT_VARTYPE_SCALAR, VISIT_VARCENTERING_NODE, Units='m')

def run_insitu(input_fname):

    csd = VOLVisIt(input_fname)
    name = 'VOL'
    description = 'VOL Visit Instrumentation'
    v = VisitInstrumentation(name, description, prefix='./', step=csd.step, cycle_time_provider=csd.cycle_time_provider, trace=False)

    csd.register(v) 
    v.run()


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser(description="Visualize VOL input and results data.")
    parser.add_argument("input_fname",  help="VOL input file")

    args = parser.parse_args()
    run_insitu(args.input_fname)
