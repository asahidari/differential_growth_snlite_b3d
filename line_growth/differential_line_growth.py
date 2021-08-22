"""
in steps s d=10 n=2
in verts_in v
in edges_in s
in seed s d=1 n=2
in framenum s d=0 n=2
in verts_boundary v
out verts_out v
out edges_out s
"""

def setup():
    
    import numpy as np
    from sverchok.data_structure import match_long_repeat
    
    class DifferentialLineGrowth:

        COHESION_FORCE = 0.0001
        SEPARATION_FORCE = 0.8
        BOUNDARY_FORCE = 0.8
        
        COHESION_DISTANCE = 0.4
        SEPARATION_DISTANCE = 0.35
        BOUNDARY_DISTANCE = 0.35
        
        COHESION_ANGLE = np.pi * 2.0
        SEPARATION_ANGLE = np.pi * 2.0
        BOUNDARY_ANGLE = np.pi * 2.0

        MIN_VEROCITY = 0.0001
        MAX_VEROCITY = 0.027
        
        MAX_LINE_LENGTH = 0.5
        
        generated_verts = []
        generated_edges = []
        is_first = True
        
        def __init__(self, framenum, verts_list, edges_list, boundary_list):
            
            if boundary_list == None:
                boundary_list = [[]]
                
            datas = match_long_repeat([verts_list, edges_list, boundary_list])
            
            for node_datas, line_datas, boundary_datas in zip(*datas):
                
                nodes = np.array(node_datas)
                lines = np.array(line_datas)
                boundaries = np.array(boundary_datas)
                
                gen_verts_dic = {}
                gen_edges_dic = {}
                
                # Set initial status
                gen_verts_dic[0] = nodes.tolist()
                gen_edges_dic[0] = lines.tolist()
                
                # Initialize node verocities with random values
                velocities = (np.random.rand(len(nodes), 3) -0.5) * 0.05
                velocities[:,2] *= 0.0

                percent_prev = 0
                for step in range(framenum-1):

                    # Grow the node tree
                    nodes, velocities, lines = self.grow(nodes, velocities, lines)
                    
                    # Differenciate
                    nodes = self.differenciate(framenum, nodes, velocities, lines, boundaries)

                    # Store results for each step                    
                    gen_verts_dic[step + 1] =  nodes.tolist()
                    gen_edges_dic[step + 1] =  lines.tolist()

                    percent = 100 * (step + 1) // steps # percent completed
                    if percent != percent_prev:
                        percent_prev = percent
                        print(str(percent).zfill(2), "%")

                # Store generated verts and edges                
                self.generated_verts.append(gen_verts_dic)
                self.generated_edges.append(gen_edges_dic)
                
        # Function to grow the node tree
        def grow(self, nodes, velocities, lines):
            
            # Extract long lines
            distances = np.array([np.linalg.norm(nodes[line[0]]-nodes[line[1]]) for line in lines])
            small_line_indices = np.where(distances < self.MAX_LINE_LENGTH)
            long_lines = np.delete(lines, small_line_indices, 0)
            
            if len(long_lines) == 0:
                return nodes, velocities, lines
            
            # Get middle points of long lines
            insert_v_indices = [line[1] for line in long_lines]
            insert_verts = [(nodes[line[0]] + nodes[line[1]]) / 2.0 for line in long_lines]

            # Set new velocities for new nodes
            if self.is_first:
                insert_velocities = [(velocities[line[0]] + velocities[line[1]]) / 2.0 for line in long_lines]
            else:
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
                            
        def differenciate(self, step, nodes, velocities, lines, boundaries):
            
            node_num = len(nodes)
            
            # Initialize cohesion and separation parameters.
            cohesion = np.zeros((node_num, 3))
            separation = np.zeros((node_num, 3))
            border = np.zeros((node_num, 3))

            for i in range(node_num):
                node = nodes[i]
                v = velocities[i]
                
                # Other node points and velocities
                other_nodes = np.delete(nodes, i, axis=0)
                other_vs = np.delete(velocities, i, axis=0)
                
                # Calc distances and angles
                distance = np.linalg.norm(other_nodes - node, axis=1)
                angle = np.arccos(np.dot(v, (other_nodes - node).T) / (np.linalg.norm(v) * np.linalg.norm((other_nodes-node), axis=1)))
                
                # Extract agents that influence the nodes
                cohesion_agents = other_nodes[(distance < self.COHESION_DISTANCE) & (angle < self.COHESION_ANGLE)]
                separation_agents = other_nodes[(distance < self.SEPARATION_DISTANCE) & (angle < self.SEPARATION_ANGLE)]
                boundary_agents = np.array([])
                if verts_boundary != None:
                    boundary_array = np.array(boundaries)
                    boundary_distance = np.linalg.norm(boundary_array - node, axis=1)
                    boundary_agents = boundary_array[(boundary_distance < self.BOUNDARY_DISTANCE)]

                # Calc cohesion, separation and boundary forces
                cohesion[i] = self.COHESION_FORCE * (np.average(cohesion_agents, axis=0) - node) if len(cohesion_agents) > 0 else 0                    
                separation[i] = self.SEPARATION_FORCE * (np.sum(node - separation_agents, axis=0)) if len(separation_agents) > 0 else 0
                border[i] = self.BOUNDARY_FORCE * (np.sum(node - boundary_agents, axis=0)) if len(boundary_agents) > 0 else 0

            # Sum up all forces to get velocities
            if self.is_first:
                velocities += cohesion + separation + border
            else:
                velocities = cohesion + separation + border
            # velocities += cohesion + separation + border
            
            self.regulate(velocities)
            
            #update position
            nodes += velocities
            return nodes
        
        # Regulate velocities
        def regulate(self, velocities):
            
            for i in range(len(velocities)):
                v_abs = np.linalg.norm(velocities[i])
                if v_abs < self.MIN_VEROCITY:
                    velocities[i] = self.MIN_VEROCITY * velocities[i] / v_abs
                elif v_abs > self.MAX_VEROCITY:
                    velocities[i] = self.MAX_VEROCITY * velocities[i] / v_abs
    
    # Start from here.
    np.random.seed(seed)
    
    if verts_in is None or edges_in is None:
        raise Exception("No data.")
    DLG = DifferentialLineGrowth(steps, verts_in, edges_in, verts_boundary)

verts_out = [gv.get(framenum) for gv in DLG.generated_verts]
edges_out = [ge.get(framenum) for ge in DLG.generated_edges]

