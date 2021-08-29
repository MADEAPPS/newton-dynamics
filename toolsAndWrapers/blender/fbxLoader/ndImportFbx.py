# --------------------------------------------------------------------------
# Copyright (c) <2003-2021> <Newton Game Dynamics>
# 
# This software is provided 'as-is', without any express or implied
# warranty. In no event will the authors be held liable for any damages
# arising from the use of this software.
# 
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely
# --------------------------------------------------------------------------


"""
This script imports a Wavefront OBJ files to Blender.

Usage:
Run this script from "File->Import" menu and then load the desired OBJ file.
Note, This loads mesh objects and materials only, nurbs and curves are not supported.

http://wiki.blender.org/index.php/Scripts/Manual/Import/wavefront_obj
"""

import array
import os
import time
import bpy
import mathutils

from bpy_extras.io_utils import unpack_list
from bpy_extras.image_utils import load_image
from bpy_extras.wm_utils.progress_report import ProgressReport


def line_value(line_split):
    """
    Returns 1 string representing the value for this line
    None will be returned if there's only 1 word
    """
    length = len(line_split)
    if length == 1:
        return None

    elif length == 2:
        return line_split[1]

    elif length > 2:
        return b' '.join(line_split[1:])


def filenames_group_by_ext(line, ext):
    """
    Splits material libraries supporting spaces, so:
    b'foo bar.mtl baz spam.MTL' -> (b'foo bar.mtl', b'baz spam.MTL')
    Also handle " chars (some software use those to protect filenames with spaces, see T67266... sic).
    """
    # Note that we assume that if there are some " in that line,
    # then all filenames are properly enclosed within those...
    start = line.find(b'"') + 1
    if start != 0:
        while start != 0:
            end = line.find(b'"', start)
            if end != -1:
                yield line[start:end]
                start = line.find(b'"', end + 1) + 1
            else:
                break
        return

    line_lower = line.lower()
    i_prev = 0
    while i_prev != -1 and i_prev < len(line):
        i = line_lower.find(ext, i_prev)
        if i != -1:
            i += len(ext)
        yield line[i_prev:i].strip()
        i_prev = i


def obj_image_load(img_data, context_imagepath_map, line, DIR, recursive, relpath):
    """
    Mainly uses comprehensiveImageLoad
    But we try all space-separated items from current line when file is not found with last one
    (users keep generating/using image files with spaces in a format that does not support them, sigh...)
    Also tries to replace '_' with ' ' for Max's exporter replaces spaces with underscores.
    Also handle " chars (some software use those to protect filenames with spaces, see T67266... sic).
    Also corrects img_data (in case filenames with spaces have been split up in multiple entries, see T72148).
    """
    filepath_parts = line.split(b' ')

    start = line.find(b'"') + 1
    if start != 0:
        end = line.find(b'"', start)
        if end != 0:
            filepath_parts = (line[start:end],)

    image = None
    for i in range(-1, -len(filepath_parts), -1):
        imagepath = os.fsdecode(b" ".join(filepath_parts[i:]))
        image = context_imagepath_map.get(imagepath, ...)
        if image is ...:
            image = load_image(imagepath, DIR, recursive=recursive, relpath=relpath)
            if image is None and "_" in imagepath:
                image = load_image(imagepath.replace("_", " "), DIR, recursive=recursive, relpath=relpath)
            if image is not None:
                context_imagepath_map[imagepath] = image
                del img_data[i:]
                img_data.append(imagepath)
                break;
        else:
            del img_data[i:]
            img_data.append(imagepath)
            break;

    if image is None:
        imagepath = os.fsdecode(filepath_parts[-1])
        image = load_image(imagepath, DIR, recursive=recursive, place_holder=True, relpath=relpath)
        context_imagepath_map[imagepath] = image

    return image


def create_materials(filepath, relpath,
                     material_libs, unique_materials,
                     use_image_search, float_func):
    """
    Create all the used materials in this obj,
    assign colors and images to the materials from all referenced material libs
    """
    from math import sqrt
    from bpy_extras import node_shader_utils

    DIR = os.path.dirname(filepath)
    context_material_vars = set()

    # Don't load the same image multiple times
    context_imagepath_map = {}

    nodal_material_wrap_map = {}

    def load_material_image(blender_material, mat_wrap, context_material_name, img_data, line, type):
        """
        Set textures defined in .mtl file.
        """
        map_options = {}

        # Absolute path - c:\.. etc would work here
        image = obj_image_load(img_data, context_imagepath_map, line, DIR, use_image_search, relpath)

        curr_token = []
        for token in img_data[:-1]:
            if token.startswith(b'-') and token[1:].isalpha():
                if curr_token:
                    map_options[curr_token[0]] = curr_token[1:]
                curr_token[:] = []
            curr_token.append(token)
        if curr_token:
            map_options[curr_token[0]] = curr_token[1:]

        map_offset = map_options.get(b'-o')
        map_scale = map_options.get(b'-s')
        if map_offset is not None:
            map_offset = tuple(map(float_func, map_offset))
        if map_scale is not None:
            map_scale = tuple(map(float_func, map_scale))

        def _generic_tex_set(nodetex, image, texcoords, translation, scale):
            nodetex.image = image
            nodetex.texcoords = texcoords
            if translation is not None:
                nodetex.translation = translation
            if scale is not None:
                nodetex.scale = scale

        # Adds textures for materials (rendering)
        if type == 'Kd':
            _generic_tex_set(mat_wrap.base_color_texture, image, 'UV', map_offset, map_scale)

        elif type == 'Ka':
            # XXX Not supported?
            print("WARNING, currently unsupported ambient texture, skipped.")

        elif type == 'Ks':
            _generic_tex_set(mat_wrap.specular_texture, image, 'UV', map_offset, map_scale)

        elif type == 'Ke':
            _generic_tex_set(mat_wrap.emission_color_texture, image, 'UV', map_offset, map_scale)
            mat_wrap.emission_strength = 1.0

        elif type == 'Bump':
            bump_mult = map_options.get(b'-bm')
            bump_mult = float(bump_mult[0]) if (bump_mult and len(bump_mult[0]) > 1) else 1.0
            mat_wrap.normalmap_strength_set(bump_mult)

            _generic_tex_set(mat_wrap.normalmap_texture, image, 'UV', map_offset, map_scale)

        elif type == 'D':
            _generic_tex_set(mat_wrap.alpha_texture, image, 'UV', map_offset, map_scale)

        elif type == 'disp':
            # XXX Not supported?
            print("WARNING, currently unsupported displacement texture, skipped.")
            # ~ mat_wrap.bump_image_set(image)
            # ~ mat_wrap.bump_mapping_set(coords='UV', translation=map_offset, scale=map_scale)

        elif type == 'refl':
            map_type = map_options.get(b'-type')
            if map_type and map_type != [b'sphere']:
                print("WARNING, unsupported reflection type '%s', defaulting to 'sphere'"
                      "" % ' '.join(i.decode() for i in map_type))

            _generic_tex_set(mat_wrap.base_color_texture, image, 'Reflection', map_offset, map_scale)
            mat_wrap.base_color_texture.projection = 'SPHERE'

        else:
            raise Exception("invalid type %r" % type)

    def finalize_material(context_material, context_material_vars, spec_colors,
                          do_highlight, do_reflection, do_transparency, do_glass):
        # Finalize previous mat, if any.
        if context_material:
            if "specular" in context_material_vars:
                # XXX This is highly approximated, not sure whether we can do better...
                # TODO: Find a way to guesstimate best value from diffuse color...
                # IDEA: Use standard deviation of both spec and diff colors (i.e. how far away they are
                #       from some grey), and apply the the proportion between those two as tint factor?
                spec = sum(spec_colors) / 3.0
                # ~ spec_var = math.sqrt(sum((c - spec) ** 2 for c in spec_color) / 3.0)
                # ~ diff = sum(context_mat_wrap.base_color) / 3.0
                # ~ diff_var = math.sqrt(sum((c - diff) ** 2 for c in context_mat_wrap.base_color) / 3.0)
                # ~ tint = min(1.0, spec_var / diff_var)
                context_mat_wrap.specular = spec
                context_mat_wrap.specular_tint = 0.0
                if "roughness" not in context_material_vars:
                    context_mat_wrap.roughness = 0.0

            # FIXME, how else to use this?
            if do_highlight:
                if "specular" not in context_material_vars:
                    context_mat_wrap.specular = 1.0
                if "roughness" not in context_material_vars:
                    context_mat_wrap.roughness = 0.0
            else:
                if "specular" not in context_material_vars:
                    context_mat_wrap.specular = 0.0
                if "roughness" not in context_material_vars:
                    context_mat_wrap.roughness = 1.0

            if do_reflection:
                if "metallic" not in context_material_vars:
                    context_mat_wrap.metallic = 1.0
            else:
                # since we are (ab)using ambient term for metallic (which can be non-zero)
                context_mat_wrap.metallic = 0.0

            if do_transparency:
                if "ior" not in context_material_vars:
                    context_mat_wrap.ior = 1.0
                if "alpha" not in context_material_vars:
                    context_mat_wrap.alpha = 1.0
                # EEVEE only
                context_material.blend_method = 'BLEND'

            if do_glass:
                if "ior" not in context_material_vars:
                    context_mat_wrap.ior = 1.5

    # Try to find a MTL with the same name as the OBJ if no MTLs are specified.
    temp_mtl = os.path.splitext((os.path.basename(filepath)))[0] + ".mtl"
    if os.path.exists(os.path.join(DIR, temp_mtl)):
        material_libs.add(temp_mtl)
    del temp_mtl

    # Create new materials
    for name in unique_materials:  # .keys()
        ma_name = "Default OBJ" if name is None else name.decode('utf-8', "replace")
        ma = unique_materials[name] = bpy.data.materials.new(ma_name)
        ma_wrap = node_shader_utils.PrincipledBSDFWrapper(ma, is_readonly=False)
        nodal_material_wrap_map[ma] = ma_wrap
        ma_wrap.use_nodes = True

    for libname in sorted(material_libs):
        # print(libname)
        mtlpath = os.path.join(DIR, libname)
        if not os.path.exists(mtlpath):
            print("\tMaterial not found MTL: %r" % mtlpath)
        else:
            # Note: with modern Principled BSDF shader, things like ambient, raytrace or fresnel are always 'ON'
            # (i.e. automatically controlled by other parameters).
            do_highlight = False
            do_reflection = False
            do_transparency = False
            do_glass = False
            spec_colors = [0.0, 0.0, 0.0]

            # print('\t\tloading mtl: %e' % mtlpath)
            context_material = None
            context_mat_wrap = None
            mtl = open(mtlpath, 'rb')
            for line in mtl:  # .readlines():
                line = line.strip()
                if not line or line.startswith(b'#'):
                    continue

                line_split = line.split()
                line_id = line_split[0].lower()

                if line_id == b'newmtl':
                    # Finalize previous mat, if any.
                    finalize_material(context_material, context_material_vars, spec_colors,
                                      do_highlight, do_reflection, do_transparency, do_glass)

                    context_material_name = line_value(line_split)
                    context_material = unique_materials.get(context_material_name)
                    if context_material is not None:
                        context_mat_wrap = nodal_material_wrap_map[context_material]
                    context_material_vars.clear()

                    spec_colors[:] = [0.0, 0.0, 0.0]
                    do_highlight = False
                    do_reflection = False
                    do_transparency = False
                    do_glass = False


                elif context_material:
                    def _get_colors(line_split):
                        # OBJ 'allows' one or two components values, treat single component as greyscale, and two as blue = 0.0.
                        ln = len(line_split)
                        if ln == 2:
                            return [float_func(line_split[1])] * 3
                        elif ln == 3:
                            return [float_func(line_split[1]), float_func(line_split[2]), 0.0]
                        else:
                            return [float_func(line_split[1]), float_func(line_split[2]), float_func(line_split[3])]

                    # we need to make a material to assign properties to it.
                    if line_id == b'ka':
                        refl =  sum(_get_colors(line_split)) / 3.0
                        context_mat_wrap.metallic = refl
                        context_material_vars.add("metallic")
                    elif line_id == b'kd':
                        context_mat_wrap.base_color = _get_colors(line_split)
                    elif line_id == b'ks':
                        spec_colors[:] = _get_colors(line_split)
                        context_material_vars.add("specular")
                    elif line_id == b'ke':
                        # We cannot set context_material.emit right now, we need final diffuse color as well for this.
                        # XXX Unsupported currently
                        context_mat_wrap.emission_color = _get_colors(line_split)
                        context_mat_wrap.emission_strength = 1.0
                    elif line_id == b'ns':
                        # XXX Totally empirical conversion, trying to adapt it
                        #     (from 0.0 - 900.0 OBJ specular exponent range to 1.0 - 0.0 Principled BSDF range)...
                        val = max(0.0, min(900.0, float_func(line_split[1])))
                        context_mat_wrap.roughness = 1.0 - (sqrt(val) / 30)
                        context_material_vars.add("roughness")
                    elif line_id == b'ni':  # Refraction index (between 0.001 and 10).
                        context_mat_wrap.ior = float_func(line_split[1])
                        context_material_vars.add("ior")
                    elif line_id == b'd':  # dissolve (transparency)
                        context_mat_wrap.alpha = float_func(line_split[1])
                        context_material_vars.add("alpha")
                    elif line_id == b'tr':  # translucency
                        print("WARNING, currently unsupported 'tr' translucency option, skipped.")
                    elif line_id == b'tf':
                        # rgb, filter color, blender has no support for this.
                        print("WARNING, currently unsupported 'tf' filter color option, skipped.")
                    elif line_id == b'illum':
                        # Some MTL files incorrectly use a float for this value, see T60135.
                        illum = any_number_as_int(line_split[1])

                        # inline comments are from the spec, v4.2
                        if illum == 0:
                            # Color on and Ambient off
                            print("WARNING, Principled BSDF shader does not support illumination 0 mode "
                                  "(colors with no ambient), skipped.")
                        elif illum == 1:
                            # Color on and Ambient on
                            pass
                        elif illum == 2:
                            # Highlight on
                            do_highlight = True
                        elif illum == 3:
                            # Reflection on and Ray trace on
                            do_reflection = True
                        elif illum == 4:
                            # Transparency: Glass on
                            # Reflection: Ray trace on
                            do_transparency = True
                            do_reflection = True
                            do_glass = True
                        elif illum == 5:
                            # Reflection: Fresnel on and Ray trace on
                            do_reflection = True
                        elif illum == 6:
                            # Transparency: Refraction on
                            # Reflection: Fresnel off and Ray trace on
                            do_transparency = True
                            do_reflection = True
                        elif illum == 7:
                            # Transparency: Refraction on
                            # Reflection: Fresnel on and Ray trace on
                            do_transparency = True
                            do_reflection = True
                        elif illum == 8:
                            # Reflection on and Ray trace off
                            do_reflection = True
                        elif illum == 9:
                            # Transparency: Glass on
                            # Reflection: Ray trace off
                            do_transparency = True
                            do_reflection = False
                            do_glass = True
                        elif illum == 10:
                            # Casts shadows onto invisible surfaces
                            print("WARNING, Principled BSDF shader does not support illumination 10 mode "
                                  "(cast shadows on invisible surfaces), skipped.")
                            pass

                    elif line_id == b'map_ka':
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'Ka')
                    elif line_id == b'map_ks':
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'Ks')
                    elif line_id == b'map_kd':
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'Kd')
                    elif line_id == b'map_ke':
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'Ke')
                    elif line_id in {b'map_bump', b'bump'}:  # 'bump' is incorrect but some files use it.
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'Bump')
                    elif line_id in {b'map_d', b'map_tr'}:  # Alpha map - Dissolve
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'D')

                    elif line_id in {b'map_disp', b'disp'}:  # displacementmap
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'disp')

                    elif line_id in {b'map_refl', b'refl'}:  # reflectionmap
                        img_data = line.split()[1:]
                        if img_data:
                            load_material_image(context_material, context_mat_wrap,
                                                context_material_name, img_data, line, 'refl')
                    else:
                        print("WARNING: %r:%r (ignored)" % (filepath, line))

            # Finalize last mat, if any.
            finalize_material(context_material, context_material_vars, spec_colors,
                              do_highlight, do_reflection, do_transparency, do_glass)
            mtl.close()


def face_is_edge(face):
    """Simple check to test whether given (temp, working) data is an edge, and not a real face."""
    face_vert_loc_indices = face[0]
    face_vert_nor_indices = face[1]
    return len(face_vert_nor_indices) == 1 or len(face_vert_loc_indices) == 2


def split_mesh(verts_loc, faces, unique_materials, filepath, SPLIT_OB_OR_GROUP):
    """
    Takes vert_loc and faces, and separates into multiple sets of
    (verts_loc, faces, unique_materials, dataname)
    """

    filename = os.path.splitext((os.path.basename(filepath)))[0]

    if not SPLIT_OB_OR_GROUP or not faces:
        use_verts_nor = any(f[1] for f in faces)
        use_verts_tex = any(f[2] for f in faces)
        # use the filename for the object name since we aren't chopping up the mesh.
        return [(verts_loc, faces, unique_materials, filename, use_verts_nor, use_verts_tex)]

    def key_to_name(key):
        # if the key is a tuple, join it to make a string
        if not key:
            return filename  # assume its a string. make sure this is true if the splitting code is changed
        elif isinstance(key, bytes):
            return key.decode('utf-8', 'replace')
        else:
            return "_".join(k.decode('utf-8', 'replace') for k in key)

    # Return a key that makes the faces unique.
    face_split_dict = {}

    oldkey = -1  # initialize to a value that will never match the key

    for face in faces:
        (face_vert_loc_indices,
         face_vert_nor_indices,
         face_vert_tex_indices,
         context_material,
         _context_smooth_group,
         context_object_key,
         _face_invalid_blenpoly,
         ) = face
        key = context_object_key

        if oldkey != key:
            # Check the key has changed.
            (verts_split, faces_split, unique_materials_split, vert_remap,
             use_verts_nor, use_verts_tex) = face_split_dict.setdefault(key, ([], [], {}, {}, [], []))
            oldkey = key

        if not face_is_edge(face):
            if not use_verts_nor and face_vert_nor_indices:
                use_verts_nor.append(True)

            if not use_verts_tex and face_vert_tex_indices:
                use_verts_tex.append(True)

        # Remap verts to new vert list and add where needed
        for loop_idx, vert_idx in enumerate(face_vert_loc_indices):
            map_index = vert_remap.get(vert_idx)
            if map_index is None:
                map_index = len(verts_split)
                vert_remap[vert_idx] = map_index  # set the new remapped index so we only add once and can reference next time.
                verts_split.append(verts_loc[vert_idx])  # add the vert to the local verts

            face_vert_loc_indices[loop_idx] = map_index  # remap to the local index

            if context_material not in unique_materials_split:
                unique_materials_split[context_material] = unique_materials[context_material]

        faces_split.append(face)

    # remove one of the items and reorder
    return [(verts_split, faces_split, unique_materials_split, key_to_name(key), bool(use_vnor), bool(use_vtex))
            for key, (verts_split, faces_split, unique_materials_split, _, use_vnor, use_vtex)
            in face_split_dict.items()]


def create_mesh(new_objects,
                use_edges,
                verts_loc,
                verts_nor,
                verts_tex,
                faces,
                unique_materials,
                unique_smooth_groups,
                vertex_groups,
                dataname,
                ):
    """
    Takes all the data gathered and generates a mesh, adding the new object to new_objects
    deals with ngons, sharp edges and assigning materials
    """

    if unique_smooth_groups:
        sharp_edges = set()
        smooth_group_users = {context_smooth_group: {} for context_smooth_group in unique_smooth_groups.keys()}
        context_smooth_group_old = -1

    fgon_edges = set()  # Used for storing fgon keys when we need to tessellate/untessellate them (ngons with hole).
    edges = []
    tot_loops = 0

    context_object_key = None

    # reverse loop through face indices
    for f_idx in range(len(faces) - 1, -1, -1):
        face = faces[f_idx]

        (face_vert_loc_indices,
         face_vert_nor_indices,
         face_vert_tex_indices,
         context_material,
         context_smooth_group,
         context_object_key,
         face_invalid_blenpoly,
         ) = face

        len_face_vert_loc_indices = len(face_vert_loc_indices)

        if len_face_vert_loc_indices == 1:
            faces.pop(f_idx)  # cant add single vert faces

        # Face with a single item in face_vert_nor_indices is actually a polyline!
        elif face_is_edge(face):
            if use_edges:
                edges.extend((face_vert_loc_indices[i], face_vert_loc_indices[i + 1])
                             for i in range(len_face_vert_loc_indices - 1))
            faces.pop(f_idx)

        else:
            # Smooth Group
            if unique_smooth_groups and context_smooth_group:
                # Is a part of of a smooth group and is a face
                if context_smooth_group_old is not context_smooth_group:
                    edge_dict = smooth_group_users[context_smooth_group]
                    context_smooth_group_old = context_smooth_group

                prev_vidx = face_vert_loc_indices[-1]
                for vidx in face_vert_loc_indices:
                    edge_key = (prev_vidx, vidx) if (prev_vidx < vidx) else (vidx, prev_vidx)
                    prev_vidx = vidx
                    edge_dict[edge_key] = edge_dict.get(edge_key, 0) + 1

            # NGons into triangles
            if face_invalid_blenpoly:
                # ignore triangles with invalid indices
                if len(face_vert_loc_indices) > 3:
                    from bpy_extras.mesh_utils import ngon_tessellate
                    ngon_face_indices = ngon_tessellate(verts_loc, face_vert_loc_indices, debug_print=bpy.app.debug)
                    faces.extend([([face_vert_loc_indices[ngon[0]],
                                    face_vert_loc_indices[ngon[1]],
                                    face_vert_loc_indices[ngon[2]],
                                    ],
                                [face_vert_nor_indices[ngon[0]],
                                    face_vert_nor_indices[ngon[1]],
                                    face_vert_nor_indices[ngon[2]],
                                    ] if face_vert_nor_indices else [],
                                [face_vert_tex_indices[ngon[0]],
                                    face_vert_tex_indices[ngon[1]],
                                    face_vert_tex_indices[ngon[2]],
                                    ] if face_vert_tex_indices else [],
                                context_material,
                                context_smooth_group,
                                context_object_key,
                                [],
                                )
                                for ngon in ngon_face_indices]
                                )
                    tot_loops += 3 * len(ngon_face_indices)

                    # edges to make ngons
                    if len(ngon_face_indices) > 1:
                        edge_users = set()
                        for ngon in ngon_face_indices:
                            prev_vidx = face_vert_loc_indices[ngon[-1]]
                            for ngidx in ngon:
                                vidx = face_vert_loc_indices[ngidx]
                                if vidx == prev_vidx:
                                    continue  # broken OBJ... Just skip.
                                edge_key = (prev_vidx, vidx) if (prev_vidx < vidx) else (vidx, prev_vidx)
                                prev_vidx = vidx
                                if edge_key in edge_users:
                                    fgon_edges.add(edge_key)
                                else:
                                    edge_users.add(edge_key)

                faces.pop(f_idx)
            else:
                tot_loops += len_face_vert_loc_indices

    # Build sharp edges
    if unique_smooth_groups:
        for edge_dict in smooth_group_users.values():
            for key, users in edge_dict.items():
                if users == 1:  # This edge is on the boundary of a group
                    sharp_edges.add(key)

    # map the material names to an index
    material_mapping = {name: i for i, name in enumerate(unique_materials)}  # enumerate over unique_materials keys()

    materials = [None] * len(unique_materials)

    for name, index in material_mapping.items():
        materials[index] = unique_materials[name]

    me = bpy.data.meshes.new(dataname)

    # make sure the list isnt too big
    for material in materials:
        me.materials.append(material)

    me.vertices.add(len(verts_loc))
    me.loops.add(tot_loops)
    me.polygons.add(len(faces))

    # verts_loc is a list of (x, y, z) tuples
    me.vertices.foreach_set("co", unpack_list(verts_loc))

    loops_vert_idx = tuple(vidx for (face_vert_loc_indices, _, _, _, _, _, _) in faces for vidx in face_vert_loc_indices)
    faces_loop_start = []
    lidx = 0
    for f in faces:
        face_vert_loc_indices = f[0]
        nbr_vidx = len(face_vert_loc_indices)
        faces_loop_start.append(lidx)
        lidx += nbr_vidx
    faces_loop_total = tuple(len(face_vert_loc_indices) for (face_vert_loc_indices, _, _, _, _, _, _) in faces)

    me.loops.foreach_set("vertex_index", loops_vert_idx)
    me.polygons.foreach_set("loop_start", faces_loop_start)
    me.polygons.foreach_set("loop_total", faces_loop_total)

    faces_ma_index = tuple(material_mapping[context_material] for (_, _, _, context_material, _, _, _) in faces)
    me.polygons.foreach_set("material_index", faces_ma_index)

    faces_use_smooth = tuple(bool(context_smooth_group) for (_, _, _, _, context_smooth_group, _, _) in faces)
    me.polygons.foreach_set("use_smooth", faces_use_smooth)

    if verts_nor and me.loops:
        # Note: we store 'temp' normals in loops, since validate() may alter final mesh,
        #       we can only set custom lnors *after* calling it.
        me.create_normals_split()
        loops_nor = tuple(no for (_, face_vert_nor_indices, _, _, _, _, _) in faces
                             for face_noidx in face_vert_nor_indices
                             for no in verts_nor[face_noidx])
        me.loops.foreach_set("normal", loops_nor)

    if verts_tex and me.polygons:
        # Some files Do not explicitely write the 'v' value when it's 0.0, see T68249...
        verts_tex = [uv if len(uv) == 2 else uv + [0.0] for uv in verts_tex]
        me.uv_layers.new(do_init=False)
        loops_uv = tuple(uv for (_, _, face_vert_tex_indices, _, _, _, _) in faces
                            for face_uvidx in face_vert_tex_indices
                            for uv in verts_tex[face_uvidx])
        me.uv_layers[0].data.foreach_set("uv", loops_uv)

    use_edges = use_edges and bool(edges)
    if use_edges:
        me.edges.add(len(edges))
        # edges should be a list of (a, b) tuples
        me.edges.foreach_set("vertices", unpack_list(edges))

    me.validate(clean_customdata=False)  # *Very* important to not remove lnors here!
    me.update(calc_edges=use_edges, calc_edges_loose=use_edges)

    # Un-tessellate as much as possible, in case we had to triangulate some ngons...
    if fgon_edges:
        import bmesh
        bm = bmesh.new()
        bm.from_mesh(me)
        verts = bm.verts[:]
        get = bm.edges.get
        edges = [get((verts[vidx1], verts[vidx2])) for vidx1, vidx2 in fgon_edges]
        try:
            bmesh.ops.dissolve_edges(bm, edges=edges, use_verts=False)
        except:
            # Possible dissolve fails for some edges, but don't fail silently in case this is a real bug.
            import traceback
            traceback.print_exc()

        bm.to_mesh(me)
        bm.free()

    # XXX If validate changes the geometry, this is likely to be broken...
    if unique_smooth_groups and sharp_edges:
        for e in me.edges:
            if e.key in sharp_edges:
                e.use_edge_sharp = True

    if verts_nor:
        clnors = array.array('f', [0.0] * (len(me.loops) * 3))
        me.loops.foreach_get("normal", clnors)

        if not unique_smooth_groups:
            me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))

        me.normals_split_custom_set(tuple(zip(*(iter(clnors),) * 3)))
        me.use_auto_smooth = True

    ob = bpy.data.objects.new(me.name, me)
    new_objects.append(ob)

    # Create the vertex groups. No need to have the flag passed here since we test for the
    # content of the vertex_groups. If the user selects to NOT have vertex groups saved then
    # the following test will never run
    for group_name, group_indices in vertex_groups.items():
        group = ob.vertex_groups.new(name=group_name.decode('utf-8', "replace"))
        group.add(group_indices, 1.0, 'REPLACE')


def create_nurbs(context_nurbs, vert_loc, new_objects):
    """
    Add nurbs object to blender, only support one type at the moment
    """
    deg = context_nurbs.get(b'deg', (3,))
    curv_range = context_nurbs.get(b'curv_range')
    curv_idx = context_nurbs.get(b'curv_idx', [])
    parm_u = context_nurbs.get(b'parm_u', [])
    parm_v = context_nurbs.get(b'parm_v', [])
    name = context_nurbs.get(b'name', b'ObjNurb')
    cstype = context_nurbs.get(b'cstype')

    if cstype is None:
        print('\tWarning, cstype not found')
        return
    if cstype != b'bspline':
        print('\tWarning, cstype is not supported (only bspline)')
        return
    if not curv_idx:
        print('\tWarning, curv argument empty or not set')
        return
    if len(deg) > 1 or parm_v:
        print('\tWarning, surfaces not supported')
        return

    cu = bpy.data.curves.new(name.decode('utf-8', "replace"), 'CURVE')
    cu.dimensions = '3D'

    nu = cu.splines.new('NURBS')
    nu.points.add(len(curv_idx) - 1)  # a point is added to start with
    nu.points.foreach_set("co", [co_axis for vt_idx in curv_idx for co_axis in (vert_loc[vt_idx] + [1.0])])

    nu.order_u = deg[0] + 1

    # get for endpoint flag from the weighting
    if curv_range and len(parm_u) > deg[0] + 1:
        do_endpoints = True
        for i in range(deg[0] + 1):

            if abs(parm_u[i] - curv_range[0]) > 0.0001:
                do_endpoints = False
                break

            if abs(parm_u[-(i + 1)] - curv_range[1]) > 0.0001:
                do_endpoints = False
                break

    else:
        do_endpoints = False

    if do_endpoints:
        nu.use_endpoint_u = True

    # close
    '''
    do_closed = False
    if len(parm_u) > deg[0]+1:
        for i in xrange(deg[0]+1):
            #print curv_idx[i], curv_idx[-(i+1)]

            if curv_idx[i]==curv_idx[-(i+1)]:
                do_closed = True
                break

    if do_closed:
        nu.use_cyclic_u = True
    '''

    ob = bpy.data.objects.new(name.decode('utf-8', "replace"), cu)

    new_objects.append(ob)


def strip_slash(line_split):
    if line_split[-1][-1] == 92:  # '\' char
        if len(line_split[-1]) == 1:
            line_split.pop()  # remove the \ item
        else:
            line_split[-1] = line_split[-1][:-1]  # remove the \ from the end last number
        return True
    return False


def get_float_func(filepath):
    """
    find the float function for this obj file
    - whether to replace commas or not
    """
    file = open(filepath, 'rb')
    for line in file:  # .readlines():
        line = line.lstrip()
        if line.startswith(b'v'):  # vn vt v
            if b',' in line:
                file.close()
                return lambda f: float(f.replace(b',', b'.'))
            elif b'.' in line:
                file.close()
                return float

    file.close()
    # in case all vert values were ints
    return float


def any_number_as_int(svalue):
    if b',' in svalue:
        svalue = svalue.replace(b',', b'.')
    return int(float(svalue))


def load(context,
         filepath,
         *,
         global_clamp_size=0.0,
         use_smooth_groups=True,
         use_edges=True,
         use_split_objects=True,
         use_split_groups=False,
         use_image_search=True,
         use_groups_as_vgroups=False,
         relpath=None,
         global_matrix=None
         ):
    """
    Called by the user interface or another script.
    load_obj(path) - should give acceptable results.
    This function passes the file and sends the data off
        to be split into objects and then converted into mesh objects
    """
    def unique_name(existing_names, name_orig):
        i = 0
        if name_orig is None:
            name_orig = b"ObjObject"
        name = name_orig
        while name in existing_names:
            name = b"%s.%03d" % (name_orig, i)
            i += 1
        existing_names.add(name)
        return name

    def handle_vec(line_start, context_multi_line, line_split, tag, data, vec, vec_len):
        ret_context_multi_line = tag if strip_slash(line_split) else b''
        if line_start == tag:
            vec[:] = [float_func(v) for v in line_split[1:]]
        elif context_multi_line == tag:
            vec += [float_func(v) for v in line_split]
        if not ret_context_multi_line:
            data.append(tuple(vec[:vec_len]))
        return ret_context_multi_line

    def create_face(context_material, context_smooth_group, context_object_key):
        face_vert_loc_indices = []
        face_vert_nor_indices = []
        face_vert_tex_indices = []
        return (
            face_vert_loc_indices,
            face_vert_nor_indices,
            face_vert_tex_indices,
            context_material,
            context_smooth_group,
            context_object_key,
            [],  # If non-empty, that face is a Blender-invalid ngon (holes...), need a mutable object for that...
        )

    with ProgressReport(context.window_manager) as progress:
        progress.enter_substeps(1, "Importing OBJ %r..." % filepath)

        if global_matrix is None:
            global_matrix = mathutils.Matrix()

        if use_split_objects or use_split_groups:
            use_groups_as_vgroups = False

        verts_loc = []
        verts_nor = []
        verts_tex = []
        faces = []  # tuples of the faces
        material_libs = set()  # filenames to material libs this OBJ uses
        vertex_groups = {}  # when use_groups_as_vgroups is true

        # Get the string to float conversion func for this file- is 'float' for almost all files.
        float_func = get_float_func(filepath)

        # Context variables
        context_material = None
        context_smooth_group = None
        context_object_key = None
        context_object_obpart = None
        context_vgroup = None

        objects_names = set()

        # Nurbs
        context_nurbs = {}
        nurbs = []
        context_parm = b''  # used by nurbs too but could be used elsewhere

        # Until we can use sets
        use_default_material = False
        unique_materials = {}
        unique_smooth_groups = {}
        # unique_obects= {} - no use for this variable since the objects are stored in the face.

        # when there are faces that end with \
        # it means they are multiline-
        # since we use xreadline we cant skip to the next line
        # so we need to know whether
        context_multi_line = b''

        # Per-face handling data.
        face_vert_loc_indices = None
        face_vert_nor_indices = None
        face_vert_tex_indices = None
        verts_loc_len = verts_nor_len = verts_tex_len = 0
        face_items_usage = set()
        face_invalid_blenpoly = None
        prev_vidx = None
        face = None
        vec = []

        quick_vert_failures = 0
        skip_quick_vert = False

        progress.enter_substeps(3, "Parsing OBJ file...")
        with open(filepath, 'rb') as f:
            for line in f:
                line_split = line.split()

                if not line_split:
                    continue

                line_start = line_split[0]  # we compare with this a _lot_

                if len(line_split) == 1 and not context_multi_line and line_start != b'end':
                    print("WARNING, skipping malformatted line: %s" % line.decode('UTF-8', 'replace').rstrip())
                    continue

                # Handling vertex data are pretty similar, factorize that.
                # Also, most OBJ files store all those on a single line, so try fast parsing for that first,
                # and only fallback to full multi-line parsing when needed, this gives significant speed-up
                # (~40% on affected code).
                if line_start == b'v':
                    vdata, vdata_len, do_quick_vert = verts_loc, 3, not skip_quick_vert
                elif line_start == b'vn':
                    vdata, vdata_len, do_quick_vert = verts_nor, 3, not skip_quick_vert
                elif line_start == b'vt':
                    vdata, vdata_len, do_quick_vert = verts_tex, 2, not skip_quick_vert
                elif context_multi_line == b'v':
                    vdata, vdata_len, do_quick_vert = verts_loc, 3, False
                elif context_multi_line == b'vn':
                    vdata, vdata_len, do_quick_vert = verts_nor, 3, False
                elif context_multi_line == b'vt':
                    vdata, vdata_len, do_quick_vert = verts_tex, 2, False
                else:
                    vdata_len = 0

                if vdata_len:
                    if do_quick_vert:
                        try:
                            vdata.append(list(map(float_func, line_split[1:vdata_len + 1])))
                        except:
                            do_quick_vert = False
                            # In case we get too many failures on quick parsing, force fallback to full multi-line one.
                            # Exception handling can become costly...
                            quick_vert_failures += 1
                            if quick_vert_failures > 10000:
                                skip_quick_vert = True
                    if not do_quick_vert:
                        context_multi_line = handle_vec(line_start, context_multi_line, line_split,
                                                        context_multi_line or line_start,
                                                        vdata, vec, vdata_len)

                elif line_start == b'f' or context_multi_line == b'f':
                    if not context_multi_line:
                        line_split = line_split[1:]
                        # Instantiate a face
                        face = create_face(context_material, context_smooth_group, context_object_key)
                        (face_vert_loc_indices, face_vert_nor_indices, face_vert_tex_indices,
                         _1, _2, _3, face_invalid_blenpoly) = face
                        faces.append(face)
                        face_items_usage.clear()
                        verts_loc_len = len(verts_loc)
                        verts_nor_len = len(verts_nor)
                        verts_tex_len = len(verts_tex)
                        if context_material is None:
                            use_default_material = True
                    # Else, use face_vert_loc_indices and face_vert_tex_indices previously defined and used the obj_face

                    context_multi_line = b'f' if strip_slash(line_split) else b''

                    for v in line_split:
                        obj_vert = v.split(b'/')
                        idx = int(obj_vert[0])  # Note that we assume here we cannot get OBJ invalid 0 index...
                        vert_loc_index = (idx + verts_loc_len) if (idx < 1) else idx - 1
                        # Add the vertex to the current group
                        # *warning*, this wont work for files that have groups defined around verts
                        if use_groups_as_vgroups and context_vgroup:
                            vertex_groups[context_vgroup].append(vert_loc_index)
                        # This a first round to quick-detect ngons that *may* use a same edge more than once.
                        # Potential candidate will be re-checked once we have done parsing the whole face.
                        if not face_invalid_blenpoly:
                            # If we use more than once a same vertex, invalid ngon is suspected.
                            if vert_loc_index in face_items_usage:
                                face_invalid_blenpoly.append(True)
                            else:
                                face_items_usage.add(vert_loc_index)
                        face_vert_loc_indices.append(vert_loc_index)

                        # formatting for faces with normals and textures is
                        # loc_index/tex_index/nor_index
                        if len(obj_vert) > 1 and obj_vert[1] and obj_vert[1] != b'0':
                            idx = int(obj_vert[1])
                            face_vert_tex_indices.append((idx + verts_tex_len) if (idx < 1) else idx - 1)
                        else:
                            face_vert_tex_indices.append(0)

                        if len(obj_vert) > 2 and obj_vert[2] and obj_vert[2] != b'0':
                            idx = int(obj_vert[2])
                            face_vert_nor_indices.append((idx + verts_nor_len) if (idx < 1) else idx - 1)
                        else:
                            face_vert_nor_indices.append(0)

                    if not context_multi_line:
                        # Means we have finished a face, we have to do final check if ngon is suspected to be blender-invalid...
                        if face_invalid_blenpoly:
                            face_invalid_blenpoly.clear()
                            face_items_usage.clear()
                            prev_vidx = face_vert_loc_indices[-1]
                            for vidx in face_vert_loc_indices:
                                edge_key = (prev_vidx, vidx) if (prev_vidx < vidx) else (vidx, prev_vidx)
                                if edge_key in face_items_usage:
                                    face_invalid_blenpoly.append(True)
                                    break
                                face_items_usage.add(edge_key)
                                prev_vidx = vidx

                elif use_edges and (line_start == b'l' or context_multi_line == b'l'):
                    # very similar to the face load function above with some parts removed
                    if not context_multi_line:
                        line_split = line_split[1:]
                        # Instantiate a face
                        face = create_face(context_material, context_smooth_group, context_object_key)
                        face_vert_loc_indices = face[0]
                        # XXX A bit hackish, we use special 'value' of face_vert_nor_indices (a single True item) to tag this
                        #     as a polyline, and not a regular face...
                        face[1][:] = [True]
                        faces.append(face)
                        if context_material is None:
                            use_default_material = True
                    # Else, use face_vert_loc_indices previously defined and used the obj_face

                    context_multi_line = b'l' if strip_slash(line_split) else b''

                    for v in line_split:
                        obj_vert = v.split(b'/')
                        idx = int(obj_vert[0]) - 1
                        face_vert_loc_indices.append((idx + len(verts_loc) + 1) if (idx < 0) else idx)

                elif line_start == b's':
                    if use_smooth_groups:
                        context_smooth_group = line_value(line_split)
                        if context_smooth_group == b'off':
                            context_smooth_group = None
                        elif context_smooth_group:  # is not None
                            unique_smooth_groups[context_smooth_group] = None

                elif line_start == b'o':
                    if use_split_objects:
                        context_object_key = unique_name(objects_names, line_value(line_split))
                        context_object_obpart = context_object_key
                        # unique_objects[context_object_key]= None

                elif line_start == b'g':
                    if use_split_groups:
                        grppart = line_value(line_split)
                        context_object_key = (context_object_obpart, grppart) if context_object_obpart else grppart
                        # print 'context_object_key', context_object_key
                        # unique_objects[context_object_key]= None
                    elif use_groups_as_vgroups:
                        context_vgroup = line_value(line.split())
                        if context_vgroup and context_vgroup != b'(null)':
                            vertex_groups.setdefault(context_vgroup, [])
                        else:
                            context_vgroup = None  # dont assign a vgroup

                elif line_start == b'usemtl':
                    context_material = line_value(line.split())
                    unique_materials[context_material] = None
                elif line_start == b'mtllib':  # usemap or usemat
                    # can have multiple mtllib filenames per line, mtllib can appear more than once,
                    # so make sure only occurrence of material exists
                    material_libs |= {os.fsdecode(f) for f in filenames_group_by_ext(line.lstrip()[7:].strip(), b'.mtl')
                    }

                    # Nurbs support
                elif line_start == b'cstype':
                    context_nurbs[b'cstype'] = line_value(line.split())  # 'rat bspline' / 'bspline'
                elif line_start == b'curv' or context_multi_line == b'curv':
                    curv_idx = context_nurbs[b'curv_idx'] = context_nurbs.get(b'curv_idx', [])  # in case were multiline

                    if not context_multi_line:
                        context_nurbs[b'curv_range'] = float_func(line_split[1]), float_func(line_split[2])
                        line_split[0:3] = []  # remove first 3 items

                    if strip_slash(line_split):
                        context_multi_line = b'curv'
                    else:
                        context_multi_line = b''

                    for i in line_split:
                        vert_loc_index = int(i) - 1

                        if vert_loc_index < 0:
                            vert_loc_index = len(verts_loc) + vert_loc_index + 1

                        curv_idx.append(vert_loc_index)

                elif line_start == b'parm' or context_multi_line == b'parm':
                    if context_multi_line:
                        context_multi_line = b''
                    else:
                        context_parm = line_split[1]
                        line_split[0:2] = []  # remove first 2

                    if strip_slash(line_split):
                        context_multi_line = b'parm'
                    else:
                        context_multi_line = b''

                    if context_parm.lower() == b'u':
                        context_nurbs.setdefault(b'parm_u', []).extend([float_func(f) for f in line_split])
                    elif context_parm.lower() == b'v':  # surfaces not supported yet
                        context_nurbs.setdefault(b'parm_v', []).extend([float_func(f) for f in line_split])
                    # else: # may want to support other parm's ?

                elif line_start == b'deg':
                    context_nurbs[b'deg'] = [int(i) for i in line.split()[1:]]
                elif line_start == b'end':
                    # Add the nurbs curve
                    if context_object_key:
                        context_nurbs[b'name'] = context_object_key
                    nurbs.append(context_nurbs)
                    context_nurbs = {}
                    context_parm = b''

                ''' # How to use usemap? deprecated?
                elif line_start == b'usema': # usemap or usemat
                    context_image= line_value(line_split)
                '''

        progress.step("Done, loading materials and images...")

        if use_default_material:
            unique_materials[None] = None
        create_materials(filepath, relpath, material_libs, unique_materials,
                         use_image_search, float_func)

        progress.step("Done, building geometries (verts:%i faces:%i materials: %i smoothgroups:%i) ..." %
                      (len(verts_loc), len(faces), len(unique_materials), len(unique_smooth_groups)))

        # deselect all
        if bpy.ops.object.select_all.poll():
            bpy.ops.object.select_all(action='DESELECT')

        new_objects = []  # put new objects here

        # Split the mesh by objects/materials, may
        SPLIT_OB_OR_GROUP = bool(use_split_objects or use_split_groups)

        for data in split_mesh(verts_loc, faces, unique_materials, filepath, SPLIT_OB_OR_GROUP):
            verts_loc_split, faces_split, unique_materials_split, dataname, use_vnor, use_vtex = data
            # Create meshes from the data, warning 'vertex_groups' wont support splitting
            #~ print(dataname, use_vnor, use_vtex)
            create_mesh(new_objects,
                        use_edges,
                        verts_loc_split,
                        verts_nor if use_vnor else [],
                        verts_tex if use_vtex else [],
                        faces_split,
                        unique_materials_split,
                        unique_smooth_groups,
                        vertex_groups,
                        dataname,
                        )

        # nurbs support
        for context_nurbs in nurbs:
            create_nurbs(context_nurbs, verts_loc, new_objects)

        view_layer = context.view_layer
        collection = view_layer.active_layer_collection.collection

        # Create new obj
        for obj in new_objects:
            collection.objects.link(obj)
            obj.select_set(True)

            # we could apply this anywhere before scaling.
            obj.matrix_world = global_matrix

        view_layer.update()

        axis_min = [1000000000] * 3
        axis_max = [-1000000000] * 3

        if global_clamp_size:
            # Get all object bounds
            for ob in new_objects:
                for v in ob.bound_box:
                    for axis, value in enumerate(v):
                        if axis_min[axis] > value:
                            axis_min[axis] = value
                        if axis_max[axis] < value:
                            axis_max[axis] = value

            # Scale objects
            max_axis = max(axis_max[0] - axis_min[0], axis_max[1] - axis_min[1], axis_max[2] - axis_min[2])
            scale = 1.0

            while global_clamp_size < max_axis * scale:
                scale = scale / 10.0

            for obj in new_objects:
                obj.scale = scale, scale, scale

        progress.leave_substeps("Done.")
        progress.leave_substeps("Finished importing: %r" % filepath)

    return {'FINISHED'}