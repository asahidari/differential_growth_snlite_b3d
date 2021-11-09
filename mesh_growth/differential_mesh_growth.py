"""
in steps s d=10 n=2
in vertices_in v
in edges_in s
in faces_in s
in weights_in s
in random_seed s d=1 n=2
in relax_iterations_in s d=2 n=2
in max_edge_length_in s d=0.8 n=2
in frame_num s d=0 n=2
out vertices_out v
out edges_out s
out faces_out s
"""

def setup():
    
    from enum import Enum
    import copy
    import numpy as np
    from bmesh.ops import subdivide_edges
    from sverchok.data_structure import match_long_repeat
    from sverchok.utils.relax_mesh import lloyd_relax, edges_relax, faces_relax, NONE, LINEAR, NORMAL, BVH, AVERAGE, MINIMUM, MAXIMUM
    from sverchok.utils.sv_bmesh_utils import bmesh_from_pydata, pydata_from_bmesh
    
    class DifferentialMeshGrowth:
        
        SEPARATION_DISTANCE = 1.16 # Ratio to the average edge length
        SEPARATION_FORCE = 0.35 # Force to separate vertices
        MAX_VELOCITY = 1.0
        COHESION_DISTANCE = 2.0
        COHESION_FORCE = 0.0001
        RELAX_WEIGHTED_ONLY = True
        
        generated_vertices = []
        generated_edges = []
        generated_faces = []
        step_num = 10
        max_edge_length = 1.0
        average_edge_length = 0.0
        relax_iterations = 1
        percent_prev = 0
        is_first = True
        
        def __init__(self, steps, v_in, e_in, f_in, w_in, max_edge_length, relax_iterations):
            self.step_num = steps
            self.max_edge_length = max_edge_length
            self.relax_iterations = relax_iterations

            datas = match_long_repeat([v_in, e_in, f_in, w_in])
            for v_datas, e_datas, f_datas, w_datas in zip(*datas):
                
                vertices = np.array(v_datas)
                edges = np.array(e_datas)
                faces = np.array(f_datas)
                weights = np.array(w_datas)
                
                gen_vertices_dic = {}
                gen_edges_dic = {}
                gen_faces_dic = {}
                
                # Set initial status
                gen_vertices_dic[0] = vertices.tolist()
                gen_edges_dic[0] = edges.tolist()
                gen_faces_dic[0] = faces.tolist()
                
                velocities = (np.random.rand(len(vertices), 3) -0.5) * 0.001
                
                percent_prev = 0
                for step in range(self.step_num):

                    print("vertices count:", len(vertices))
                    vertices, edges, faces, velocities, weights = self.process(vertices, edges, faces, velocities, weights)

                    self.print_progress(step)

                    # Store results for each step                    
                    gen_vertices_dic[step + 1] =  vertices.tolist()
                    gen_edges_dic[step + 1] =  edges.tolist()
                    gen_faces_dic[step + 1] =  faces.tolist()

                # Store generated vertices, edges and faces
                self.generated_vertices.append(gen_vertices_dic)
                self.generated_edges.append(gen_edges_dic)
                self.generated_faces.append(gen_faces_dic)

        # Process (main function)
        def process(self, vertices, edges, faces, velocities, weights):
            
            vertices = self.relax_mesh(vertices, edges, faces, weights)

            vertices, edges, faces = self.subdivide_long_edges(vertices, edges, faces)
            
            velocities, weights = self.compute_velocities(vertices, edges, velocities, weights)
            vertices += velocities
            
            return vertices, edges, faces, velocities, weights

        # Relax mesh
        def relax_mesh(self, vertices, edges, faces, weights):
            
            algorithm = 'LLOYD' # or 'EDGES' or 'FACES'
            target = AVERAGE # or MINIMUM or MAXIMUM
            preserve_shape = NONE # or LINEAR or NORMAL or BVH
            
            factor = 0.5
            # mask = [[1]]
            mask = [w > 0 for w in weights] if self.RELAX_WEIGHTED_ONLY else [[1]] 
            skip_bounds = True
            used_axes = {0, 1, 2}
            
            new_vertices = []
            if algorithm == 'LLOYD':
                new_vertices = lloyd_relax(vertices.tolist(), faces.tolist(), self.relax_iterations,
                                mask = mask,
                                method = preserve_shape,
                                skip_boundary = skip_bounds,
                                use_axes = used_axes)
            elif algorithm == 'EDGES':
                new_vertices = edges_relax(vertices.tolist(), edges.tolist(), faces.tolist(), iterations,
                                k = factor,
                                mask = mask,
                                method = preserve_shape,
                                target = target,
                                skip_boundary = skip_bounds,
                                use_axes = used_axes)
            elif algorithm == 'FACES':
                new_vertices = faces_relax(vertices.tolist(), edges.tolist(), faces.tolist(), iterations,
                                k = factor,
                                mask = mask,
                                method = preserve_shape,
                                target = target,
                                skip_boundary = skip_bounds,
                                use_axes = used_axes)
            
            return np.array(new_vertices)

        # Subdivide mesh
        def subdivide_long_edges(self, vertices, edges, faces):
            
            bm = bmesh_from_pydata(
                vertices, edges, faces,
                markup_face_data=False,
                markup_edge_data=True,
                normal_update=True)
            
            edge_lengths = np.array([e.calc_length() for e in bm.edges])
            mask = (edge_lengths > self.max_edge_length).tolist()
            selected_edges = []
            for be in bm.edges:
                if be.calc_length() > self.max_edge_length:
                    selected_edges.append(be)
            # edge_id = bm.edges.layers.int.get("initial_index")            
            # selected_edges = [edge for edge in bm.edges if mask[edge[edge_id]]]
            
            if len(selected_edges) == 0:
                return vertices, edges, faces
            
            geom = subdivide_edges(
                bm,
                edges=selected_edges,
                smooth=0.8,
                smooth_falloff='SMOOTH',
                fractal=0.0,
                along_normal=0.0,
                cuts=1,
                seed=1,
                quad_corner_type='STRAIGHT_CUT',
                use_grid_fill=True,
                use_single_edge=True,
                use_only_quads=False,
                use_smooth_even=False)
                
            new_vertices, new_edges, new_faces = pydata_from_bmesh(bm)
            bm.free()
            
            return np.array(new_vertices), np.array(new_edges), np.array(new_faces)

        # Compute forces to move vertices on the basis of vertex weights
        def compute_velocities(self, vertices, edges, velocities, weights):
            
            # velocities = self.adjust_velocities(vertices, edges, velocities)
            velocities = self.interpolate_vertex_values(vertices, edges, velocities, [0, 0, 0])
            weights = self.interpolate_vertex_values(vertices, edges, weights, 0.0)
            
            vertices_num = len(vertices)
            cohesion = np.zeros((vertices_num, 3))
            separation = np.zeros((vertices_num, 3))
            
            self.average_edge_length = np.average(np.linalg.norm(vertices[edges[:,0]] - vertices[edges[:,1]], axis=1))
            
            for i in range(vertices_num):
                
                other_vertices = np.delete(vertices, i, axis=0)
                distances = np.linalg.norm(other_vertices - vertices[i], axis=1)
                
                cohesion_agents = other_vertices[(distances < self.average_edge_length * self.COHESION_DISTANCE)]
                separation_agents = other_vertices[(distances < self.average_edge_length * self.SEPARATION_DISTANCE)]
                
                cohesion[i] = self.COHESION_FORCE * (np.average(cohesion_agents, axis=0) - vertices[i]) if len(cohesion_agents) > 0 else 0
                separation[i] = self.SEPARATION_FORCE * (np.sum(vertices[i] - separation_agents, axis=0)) if len(separation_agents) > 0 else 0

            weight_vec = np.repeat(weights[:, None], 3, axis=1) + 0.01
            if self.is_first:
                velocities += (cohesion + separation) * weight_vec
            else:
                velocities = (cohesion + separation) * weight_vec
            
            self.is_first = False
            return self.regulate_velocities(velocities), weights

        # Interporate vertex values
        def interpolate_vertex_values(self, vertices, edges, values, default_value):
            if len(values) < len(vertices):
                indices = np.arange(len(values), len(vertices))
                new_values = []
                for i in indices:
                    edge_indices = np.where(np.logical_or(edges[:,0]==i, edges[:,1]==i))[0]
                    paired_v_idx = [edges[ei][1] if edges[ei][0]==i else edges[ei][0] for ei in edge_indices]
                    neighbor_values = [values[vi] if vi < len(values) else default_value for vi in paired_v_idx]
                    new_value = np.mean(neighbor_values, axis=0)
                    if isinstance(default_value, list):
                        new_value = new_value.tolist()
                    new_values.append(new_value)
                values = np.append(values, np.array(new_values), axis=0)
            return values
                        
        # Regulate velocities
        def regulate_velocities(self, velocities):
            return np.where(velocities > self.MAX_VELOCITY, self.MAX_VELOCITY, velocities)

        # Print progress
        def print_progress(self, step):
            percent = 100 * (step + 1) // steps # percent completed
            if percent != self.percent_prev:
                self.percent_prev = percent
                print(str(percent).zfill(2), "%")

        # Get frame datas
        def get_frame(self, frame_number):
            v_out = [gv.get(frame_number) for gv in self.generated_vertices]
            e_out = [ge.get(frame_number) for ge in self.generated_edges]
            f_out = [gf.get(frame_number) for gf in self.generated_faces]
            return v_out, e_out, f_out

    if vertices_in == None or edges_in == None or faces_in == None or weights_in == None \
        or len(vertices_in) == 0 or len(edges_in) == 0 or len(faces_in) == 0 or len(weights_in) == 0:
        return
    
    np.random.seed(random_seed)
    DFMG = DifferentialMeshGrowth(steps, vertices_in, edges_in, faces_in, weights_in, max_edge_length_in, relax_iterations_in)

vertices_out, edges_out, faces_out = DFMG.get_frame(frame_num)

