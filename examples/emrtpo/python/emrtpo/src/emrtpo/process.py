# -*- coding: utf-8 -*-

import os
import subprocess
import uuid
from pathlib import Path


try:
    _origin_binary = os.environ['ORIGIN_BINARY']
except:
    _origin_binary = None

try:
    _emrtpo_tracer_binary = os.environ['EMRTPO_TRACER_BINARY']
except:
    _emrtpo_tracer_binary = None

try:
    _emrtpo_pattern_processor_binary = os.environ['EMRTPO_FLD_PO_BINARY']
except:
    _emrtpo_pattern_processor_binary = None

try:
    _emrtpo_cwd = os.environ['EMRTPO_CWD']
except:
    _emrtpo_cwd = "./"


def get_temp_prefix():
    global _emrtpo_cwd
    return str(Path(_emrtpo_cwd).joinpath(str(uuid.uuid4())))


def run_origin(filename):
    global _origin_binary
    global _emrtpo_cwd
    if _origin_binary:
        p = subprocess.Popen([_origin_binary, filename], cwd=_emrtpo_cwd)
        p.wait()
        return p.communicate()


def run_emrtpo_tracer(filename):
    global _emrtpo_tracer_binary
    global _emrtpo_cwd
    if _emrtpo_tracer_binary:
        p = subprocess.Popen([_emrtpo_tracer_binary, filename], cwd=_emrtpo_cwd)
        p.wait()
        return p.communicate()


def run_emrtpo_fld_po(filename):
    global _emrtpo_pattern_processor_binary
    global _emrtpo_cwd
    if _emrtpo_pattern_processor_binary:
        p = subprocess.Popen([_emrtpo_pattern_processor_binary, filename], cwd=_emrtpo_cwd)
        p.wait()
        return p.communicate()

