# -*- coding: utf-8 -*-

import numpy as np
from tqdm import tqdm


header_t = np.dtype([('isize', '<u1'),
               ('fsize', '<u1'), 
               ('major', '<u2'),
               ('minor', '<u2'),
               ('revision', '<u2'),
               ('nul0', '<u8'),
               ('nul1', '<u8'),
               ('nul2', '<u8'),
               ('nul3', '<u8'),
               ('nul4', '<u8'),
               ('nul5', '<u8'),
               ('nul6', '<u8'),])


n_rays_t = np.dtype([('n_rays', '<u8')])
record_t = np.dtype([('origin_id', '<u8'),
                   ('endpoint_id', '<u8'),
                   ('shot', '<u8'),
                   ('origin_u', '<f8'),
                   ('origin_v', '<f8'),
                   ('propagation_length', '<f8'),
                   ('geometric_length', '<f8'),
                   ('attenuation', '<f8'),
                   ('origin', ('<f8', 3)),
                  ])


n_endpoints_t = np.dtype([('n_endpoints', '<u8')])
n_kinds_t = np.dtype([('n_kinds', '<u8')])
n_iors_t = np.dtype([('n_iorss', '<u8')])
n_atts_t = np.dtype([('n_attenuations', '<u8')])


def read_emrtpo_record(b, offset):
    record = np.frombuffer(b[offset:offset + 11*8], record_t)
    offset += 11*8

    n_endpoints = int(np.frombuffer(b[offset:offset + 8], n_endpoints_t)[0][0])
    offset += 8

    endpoints_t = np.dtype([('endpoints',('<f8', (n_endpoints, 3)))])
    if n_endpoints>0:
        endpoints = np.frombuffer(b[offset:offset+n_endpoints*24], endpoints_t)[0][0]
        offset += n_endpoints*24
    else:
        endpoints = list()

    n_kinds = int(np.frombuffer(b[offset:offset + 8], n_kinds_t)[0][0])
    offset += 8

    if n_kinds > 0:
        kinds_t = np.dtype([('kinds',('<u1', n_kinds))])
        kinds = np.frombuffer(b[offset:offset + n_kinds], kinds_t)
        offset += n_kinds
    else:
        kinds = list()

    n_iors = int(np.frombuffer(b[offset:offset + 8], n_iors_t)[0][0])
    offset += 8

    if n_iors > 0:
        iors_t = np.dtype([('iors',('<f8', n_iors))])
        iors = np.frombuffer(b[offset:offset + n_iors*8], iors_t)
        offset += n_iors*8
    else:
        iors = list()

    n_atts = int(np.frombuffer(b[offset:offset + 8], n_atts_t)[0][0])
    offset += 8

    if n_iors > 0:
        atts_t = np.dtype([('attenuations',('<f8', n_atts))])
        atts = np.frombuffer(b[offset:offset + n_atts*8], atts_t)
        offset += n_atts*8
    else:
        atts = list()

    return record, endpoints, kinds, offset, iors, atts


def read_and_process_binary_emrtpo_data(fname, processor=None, collect_raypaths=False):

    raypaths = list() 

    with open(fname, 'rb') as f:
        b = f.read()
        header = np.frombuffer(b[:64], header_t)
        offset = 64

        for i in tqdm(range(int(np.frombuffer(b[-8:], n_rays_t)[0][0]))):
            r, e, k, offset, iors, atts = read_emrtpo_record(b, offset)
            if processor:  # Processor object may be an aggregator that, e.g., computes the received signal or generates the meshed for visualization
                processor(r, e, k)

            if collect_raypaths:
                raypaths.append((r, e, k))

    return raypaths
