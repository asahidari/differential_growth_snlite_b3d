"""
in verts_in v
in faces_in s
in selected_verts_in v
in selected_edges_in s
in steps s d=10 n=2
in seed s d=5 n=2
in framenum s d=0 n=2
out verts_out v
out edges_out s
"""

def setup():
    
    import numpy as np
    from enum import Enum
    from mathutils import Vector
    from mathutils.bvhtree import BVHTree
    from sverchok.data_structure import match_long_repeat
    
    class GrowOnMesh:
        
        COHESION_FORCE = 0.0001
        SEPARATION_FORCE = 0.46
        
        COHESION_DISTANCE = 0.22
        SEPARATION_DISTANCE = 0.12

        MIN_VELOCITY = 0.0001
        MAX_VELOCITY = 0.006

        MAX_LINE_LENGTH = 0.24
        
        generated_verts = []
        generated_edges = []
 
        progress_percent = 0
        bvhtree = None
                
        def __init__(self, step_num, bvhtree, selected_verts, selected_edges):
            
            if bvhtree is None or selected_verts is None or selected_edges is None:
                print("Some inputs are not yet connected.")
                return
            
            datas = match_long_repeat([selected_verts, selected_edges])
            
            for node_datas, line_datas in zip(*datas):
                
                nodes = np.array(node_datas)
                lines = np.array(line_datas)

                self.bvhtree = bvhtree
                                
                gen_verts_dic = { 0 : nodes.tolist() }
                gen_edges_dic = { 0 : lines.tolist() }
                
                velocities = (np.random.rand(len(nodes), 3) -0.5) * 0.01
                # velocities = np.zeros((len(nodes), 3), dtype=np.float64)
                
                for step in range(step_num-1):
                    
                    # Grow the node tree
                    nodes, velocities, lines = self.grow(nodes, lines, velocities)
                    
                    # Calculate forces between nodes
                    velocities = self.calc_forces(nodes, velocities)
                    
                    # Find node positions on surface
                    nodes = self.apply_on_surface(nodes, velocities)
                    
                    gen_verts_dic[step + 1] = nodes.tolist()
                    gen_edges_dic[step + 1] = lines.tolist()
                    
                    # Print progress
                    self.print_progress(step, steps)
                
                # Store generated verts and edges
                self.generated_verts.append(gen_verts_dic)
                self.generated_edges.append(gen_edges_dic)
                
                print("Done!")
            
        # Function to grow the node tree
        def grow(self, nodes, lines, velocities):

            # Extract long lines
            distances = np.array([np.linalg.norm(nodes[line[0]]-nodes[line[1]]) for line in lines])
            small_line_indices = np.where(distances < self.MAX_LINE_LENGTH)
            long_lines = np.delete(lines, small_line_indices, 0)
            
            if len(long_lines) == 0:
                return nodes, velocities, lines
            
            # Get middle points of long lines
            insert_v_indices = [line[1] for line in long_lines]
            middle_verts = [(nodes[line[0]] + nodes[line[1]]) / 2.0 for line in long_lines]
            insert_verts = [self.bvhtree.find_nearest(middle)[0] for middle in middle_verts]

            # Set new velocities for new nodes
            # insert_velocities = [(velocities[line[0]] + velocities[line[1]]) / 2.0 for line in long_lines]
            insert_velocities = [np.zeros(3, dtype=np.float64) for line in long_lines]

            # Insert new nodes and velocities
            nodes = np.insert(nodes, insert_v_indices, insert_verts, 0)
            velocities = np.insert(velocities, insert_v_indices, insert_velocities, 0)
            
            # Modify end of lines
            prev_line_len = len(lines)
            next_line_len = prev_line_len + len(insert_v_indices)
            lines = np.delete(lines, len(lines)-1, 0)
            tail_lines = np.array([[p, (p+1) % next_line_len] \
                                for p in range(prev_line_len-1, next_line_len)])
            lines = np.vstack((lines, tail_lines))
            
            return nodes, velocities, lines

        # def calc_velocities(self, nodes, lines, velocities):
        def calc_forces(self, nodes, velocities):
                                
            node_num = len(nodes)

            # Initialize cohesion and separation parameters.
            cohesion = np.zeros((node_num, 3))
            separation = np.zeros((node_num, 3))
            
            for i in range(node_num):
                node = nodes[i]
                
                # Other nodes
                other_nodes = np.delete(nodes, i, axis=0)
                
                # Calc distances and angles
                distance = np.linalg.norm(other_nodes - node, axis=1)
    
                # Extract agents that influence the nodes
                cohesion_agents = other_nodes[distance < self.COHESION_DISTANCE]
                separation_agents = other_nodes[distance < self.SEPARATION_DISTANCE]

                # Calc cohesion and separation forces
                cohesion[i] = self.COHESION_FORCE * (np.average(cohesion_agents, axis=0) - node) \
                                                        if len(cohesion_agents) > 0 else 0
                separation[i] = self.SEPARATION_FORCE * (np.sum((node - separation_agents), axis=0)) \
                                                        if len(separation_agents) > 0 else 0
            
            # Add forces to generate velocities
            velocities += cohesion + separation
            # if step == 0:
            #     velocities += cohesion + separation
            # else:
            #     velocities = cohesion + separation
            
            for i in range(node_num):
                # Regulate velocities
                velocities[i] = self.regulate(velocities[i])

            return velocities

        # Ray cast velocity's end points from each nodes on mesh
        def apply_on_surface(self, nodes, velocities):

            # Get result points on mesh, using node + velocity vectors as ray start points
            ray_starts = nodes + velocities
            results = np.array([self.bvhtree.find_nearest(point) for point in ray_starts])
            
            new_nodes = np.array([result[0] if result[0] is not None else nodes[i] \
                            for i, result in enumerate(results)])

            return new_nodes

        # Regulate velocity speed
        def regulate(self, velocity):
            v_abs = np.linalg.norm(velocity)
            if v_abs < self.MIN_VELOCITY:
                velocity = self.MIN_VELOCITY * velocity / v_abs
            elif v_abs > self.MAX_VELOCITY:
                velocity = self.MAX_VELOCITY * velocity / v_abs
            return velocity

        # Print progress
        def print_progress(self, step, total):
            percent = 100 * (step + 1) // total # percent completed
            if percent != self.progress_percent:
                self.progress_percent = percent
                print(str(percent).zfill(2), "%")

    np.random.seed(seed)
    
    # Create BVHTree for the target polygon
    if verts_in is not None and faces_in is not None:
        vt = [tuple(v) for v in verts_in[0]]
        ft = [tuple(f) for f in faces_in[0]]
        bvhtree = BVHTree.FromPolygons(vt, ft)

    GOM = GrowOnMesh(steps, bvhtree, selected_verts_in, selected_edges_in)
    
verts_out = [verts_dic.get(framenum) for verts_dic in GOM.generated_verts]
edges_out = [edges_dic.get(framenum) for edges_dic in GOM.generated_edges]

