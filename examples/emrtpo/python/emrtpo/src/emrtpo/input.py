# -*- coding: utf-8 -*-

import os
import numpy as np
import json
from pathlib import Path
import trimesh as tm
import matplotlib.pyplot as plt
from landingsite import create_ellipsoid, simple_assign_materials, simple_assign_layers
from emrt.process import run_origin, get_temp_prefix
from tqdm import tqdm


class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)


def write_emrtpo_input_config_file(identifier, src_origin, src_radius, src_range, prefix='./',
                         shots_per_path=1000, max_bounces=100, min_bounces=3, max_attenuation=1e-9, 
                         max_propagation_length=100., termination_probability=1e-6, min_distance=1e-2,
                         binary_output=True, output_all_rays=False, seed=0, spheres=False):

    p = Path(prefix)

    md = dict()
    if binary_output:
        md["results_file"] = str(p.joinpath(identifier+'_results.raypaths'))
        md["PO_results_file"] = str(p.joinpath(identifier+'_results.po'))
    else:
        md["results_file"] = str(p.joinpath(identifier+'_results.raypaths.json'))
        md["PO_results_file"] = str(p.joinpath(identifier+'_results.po.json'))

        
    if spheres:
        spheres_centers_file = str(p.joinpath(identifier+'_spheres_centers.npy'))
        spheres_radii_file = str(p.joinpath(identifier+'_spheres_radii.npy'))
    else:
        spheres_centers_file = ""
        spheres_radii_file = ""

    md["points_file"] = str(p.joinpath(identifier+'_points.npy'))
    md["tris_file"] = str(p.joinpath(identifier+'_tris.npy'))
    md["spheres_centers_file"] = spheres_centers_file
    md["spheres_radii_file"] = spheres_radii_file
    md["ids_file"] = str(p.joinpath(identifier+'_ids.npy'))
    md["materials_file"] = str(p.joinpath(identifier+'_materials.npy'))
    md["origin"] = src_origin
    md["origin_sphere_range"] = src_range 
    md["origin_sphere_radius"] = src_radius
    md["target"] = src_origin
    md["target_sphere_range"] = src_range 
    md["target_sphere_radius"] = src_radius
    md["shots_per_path"] = shots_per_path
    md["max_bounces"] = max_bounces
    md["min_bounces"] = min_bounces
    md["max_attenuation"] = max_attenuation
    md["max_propagation_length"] = max_propagation_length
    md["termination_probability"] = termination_probability
    md["min_distance"] = min_distance
    md["binary_output"] = binary_output
    md["output_all_rays"] = output_all_rays
    md["seed"] = seed
    
    json.dump(md, open(p.joinpath(f'{identifier}.json'), 'w'), indent=2)
    
    return md


def write_emrt_input_data_files(vertices, faces, ids, materials, md, spheres_centers=None, spheres_radii=None):
    
    np.save(md['points_file'], np.ascontiguousarray(vertices, dtype=np.float64))
    np.save(md['tris_file'], np.ascontiguousarray(faces, dtype=np.uint32))
    np.save(md['ids_file'], np.ascontiguousarray(ids, dtype=np.uint32))
    np.save(md['materials_file'], np.ascontiguousarray(materials, np.float64))
    
    if spheres_centers and spheres_radii:
        np.save(md['spheres_centers_file'], np.ascontiguousarray(spheres_centers, np.float64))
        np.save(md['spheres_radii_file'], np.ascontiguousarray(spheres_radii, np.float64))


def write_origin_input_file(identifier, origin, direction, radius, cut_angle, resolution=3):

    config_data = dict()
    config_data['points_file'] = identifier + '_vertices.npy'
    config_data['tris_file'] = identifier + '_tris.npy'
    config_data['level'] = resolution
    config_data['cut_angle'] = cut_angle
    config_data['radius'] = radius
    config_data['origin'] = origin
    config_data['direction'] = direction

    json.dump(config_data, open(identifier + '.json', 'w'), indent=2, cls=NumpyEncoder)

    return config_data


def generate_emrt_input(identifier, obj, materials, src_origin, src_radius=2., src_resolution=6, src_mesh=None,
                        write_obj=False, write=True, prefix='./', simulation_config=None,
                        spheres_centers=None, spheres_radii=None, spheres_ids=None):
    
    # Top layer is first object

    if simulation_config is None:
        simulation_config = {}

    final = list()
    offsets = list()
    src_offset = 0
    
    # Compute and fix offsets
    for o in obj:
        final.append((o[0].copy(), o[1].copy() + src_offset, o[2].copy()))
        offsets.append(src_offset + o[0].shape[0])
        src_offset += o[0].shape[0]

    if not src_mesh: #TODO: This should be handled by origin
        sv, sf, si = create_ellipsoid(src_origin, 3*[src_radius, ], 0, 0, 0, 0, resolution=src_resolution)
    else:
        sv, sf, si = src_mesh
    # vertices, faces, ids, axis, threshold, above_material, below_material, op='gt'
    sv, sf, si = simple_assign_materials(sv, sf, si, 2, final[0][0][0, 2], final[0][2][0, 3], final[0][2][0, 3], op='lt')
    sv, sf, si = simple_assign_layers(sv, sf, si, 2, final[0][0][0, 2], final[0][2][0, 1], final[0][2][0, 1], op='lt')

    final.append((sv, sf + src_offset, si))
    
    v_list = list()
    f_list = list()
    i_list = list()

    for o in final:
        v_list.append(o[0])
        f_list.append(o[1])
        i_list.append(o[2])

    vertices = np.concatenate(v_list, axis=0)
    faces = np.concatenate(f_list, axis=0)
    ids = np.concatenate(i_list, axis=0)
    
    if write_obj:
        mesh = tm.Trimesh(vertices=vertices, faces=faces)
        tm.exchange.export.export_mesh(mesh, open(Path(prefix).joinpath(identifier + '.obj'), 'w'), 'obj')

    md = dict()

    if write:
        if spheres_centers and spheres_radii and spheres_ids:
            ids = np.concatenate([ids, spheres_ids], axis=0)
            simulation_config['spheres'] = True
            
        md = write_emrt_input_config_file(identifier, src_origin, src_radius, [faces.shape[0] - sf.shape[0], faces.shape[0]],
                                  prefix, **simulation_config)
        write_emrt_input_data_files(vertices, faces, ids, materials, md, spheres_centers, spheres_radii)

    return vertices, faces, ids, md


def simple_source_position_sweep(identifier, obj, materials, src_origins, src_radius=2., src_resolution=6,
                                 write_obj=False, write=True, prefix='./', simulation_config=None, src_meshes=None):

    if simulation_config is None:
        simulation_config = {}

    for k, src_origin in enumerate(tqdm(src_origins)):
        src_mesh = None
        if src_meshes:
            src_mesh = src_meshes[k]
        generate_emrt_input(f'{identifier}_{k}', obj, materials, src_origin, src_radius, src_resolution, src_mesh=src_mesh,
                            write_obj=write_obj, write=write, prefix=prefix, simulation_config=simulation_config)


def generate_minimum_source_mesh(origin, target, radius, bounding_sphere_radius, resolution: int = 3, cleanup=True):

    prefix = get_temp_prefix()

    direction = target - origin
    length = np.linalg.norm(direction)
    direction /= length

    cut_angle = 180./np.pi * np.arcsin(bounding_sphere_radius/length)

    write_origin_input_file(prefix, origin, direction, radius, cut_angle, resolution)
    run_origin(prefix + '.json')

    vertices = np.load(prefix + '_vertices.npy')
    tris = np.load(prefix + '_tris.npy')
    ids = np.zeros((tris.shape[0], 4), dtype=np.uint32)

    if cleanup:
        try:
            os.remove(prefix + '.json')
            os.remove(prefix + '_vertices.npy')
            os.remove(prefix + '_tris.npy')
        except:
            pass

    return vertices, tris, ids


def check_setup(vertices, faces, ids, s=0.1, layer=None, layer_position=0., cmp='lt', fname=None):
    v = (vertices[faces[:, 0]] + vertices[faces[:, 1]] + vertices[faces[:, 2]]) / 3.

    fig, ax = plt.subplots(3, 4)
    fig.set_size_inches(15, 15)

    def test(x, y, layer, layer_position, cmp):
        if layer and x != layer and y != layer:
            if cmp == 'gt':
                return v[:, layer] > layer_position
            else:
                return v[:, layer] < layer_position
        else:
            return np.ones(v.shape[0], dtype=bool)

    t = test(0, 1, layer, layer_position, cmp)
    # X-Y-layer above
    ax[0][0].scatter(v[:, 0][t], v[:, 1][t], c=ids[:, 0][t], s=s)
    ax[0][0].set_xlabel('x [m]')
    ax[0][0].set_ylabel('y [m]')
    ax[0][0].set_title('X-Y-Layer-Above')
    ax[0][0].set_aspect('equal', 'datalim')
    # X-Y-layer below
    ax[0][1].scatter(v[:, 0][t], v[:, 1][t], c=ids[:, 1][t], s=s)
    ax[0][1].set_xlabel('x [m]')
    ax[0][1].set_ylabel('y [m]')
    ax[0][1].set_title('X-Y-Layer-Below')
    ax[0][1].set_aspect('equal', 'datalim')
    # X-Y-Above
    ax[0][2].scatter(v[:, 0][t], v[:, 1][t], c=ids[:, 2][t], s=s)
    ax[0][2].set_xlabel('x [m]')
    ax[0][2].set_ylabel('y [m]')
    ax[0][2].set_title('X-Y-Above')
    ax[0][2].set_aspect('equal', 'datalim')
    # X-Y-Below
    ax[0][3].scatter(v[:, 0][t], v[:, 1][t], c=ids[:, 3][t], s=s)
    ax[0][3].set_xlabel('x [m]')
    ax[0][3].set_ylabel('y [m]')
    ax[0][3].set_title('X-Y-Below')
    ax[0][3].set_aspect('equal', 'datalim')

    t = test(0, 2, layer, layer_position, cmp)
    # X-Z-layer above
    ax[1][0].scatter(v[:, 0][t], v[:, 2][t], c=ids[:, 0][t], s=s)
    ax[1][0].set_xlabel('x [m]')
    ax[1][0].set_ylabel('z [m]')
    ax[1][0].set_title('X-Z-Layer-Above')
    ax[1][0].set_aspect('equal', 'datalim')
    # X-Z-layer below
    ax[1][1].scatter(v[:, 0][t], v[:, 2][t], c=ids[:, 1][t], s=s)
    ax[1][1].set_xlabel('x [m]')
    ax[1][1].set_ylabel('z [m]')
    ax[1][1].set_title('X-Z-Layer-Below')
    ax[1][1].set_aspect('equal', 'datalim')
    # X-Z-Above
    ax[1][2].scatter(v[:, 0][t], v[:, 2][t], c=ids[:, 2][t], s=s)
    ax[1][2].set_xlabel('x [m]')
    ax[1][2].set_ylabel('z [m]')
    ax[1][2].set_title('X-Z-Above')
    ax[1][2].set_aspect('equal', 'datalim')
    # X-Z-Below
    ax[1][3].scatter(v[:, 0][t], v[:, 2][t], c=ids[:, 3][t], s=s)
    ax[1][3].set_xlabel('x [m]')
    ax[1][3].set_ylabel('z [m]')
    ax[1][3].set_title('X-Z-Below')
    ax[1][3].set_aspect('equal', 'datalim')

    t = test(1, 1, layer, layer_position, cmp)
    # Y-Z-layer above
    ax[2][0].scatter(v[:, 1][t], v[:, 2][t], c=ids[:, 0][t], s=s)
    ax[2][0].set_xlabel('y [m]')
    ax[2][0].set_ylabel('z [m]')
    ax[2][0].set_title('Y-Z-Layer-Above')
    ax[2][0].set_aspect('equal', 'datalim')
    # Y-Z-layer below
    ax[2][1].scatter(v[:, 1][t], v[:, 2][t], c=ids[:, 1][t], s=s)
    ax[2][1].set_xlabel('y [m]')
    ax[2][1].set_ylabel('z [m]')
    ax[2][1].set_title('Y-Z-Layer-Below')
    ax[2][1].set_aspect('equal', 'datalim')
    # Y-Z-Above
    ax[2][2].scatter(v[:, 1][t], v[:, 2][t], c=ids[:, 2][t], s=s)
    ax[2][2].set_xlabel('y [m]')
    ax[2][2].set_ylabel('z [m]')
    ax[2][2].set_title('Y-Z-Above')
    ax[2][2].set_aspect('equal', 'datalim')
    # Y-Z-Below
    ax[2][3].scatter(v[:, 1][t], v[:, 2][t], c=ids[:, 3][t], s=s)
    ax[2][3].set_xlabel('x [m]')
    ax[2][3].set_ylabel('y [m]')
    ax[2][3].set_title('Y-Z-Below')
    ax[2][3].set_aspect('equal', 'datalim')

    fig.tight_layout()

    if fname:
        fig.savefig(fname, dpi=150)

