import simpy
import networkx as nx

#PROFESSIONAL ALTERNATIVES: UWSim, ns-3, A, UWSim-NET: UWSim with Network Simulator, DESERT underwater framework v2

class Node:
    def __init__(self, env, node_id, time_slot, transmission_latency):
        self.env = env
        self.node_id = node_id
        self.time_slot = time_slot
        self.transmission_latency = transmission_latency
        self.channel = simpy.Resource(env, capacity=1)
        
    def transmit(self):
        print(f"Node {self.node_id} transmitting in time slot {self.time_slot} at time {self.env.now}")
        yield self.env.timeout(self.transmission_latency)  # Transmission latency
        
    def run(self):
        while True:
            with self.channel.request() as req:
                yield req
                yield self.env.process(self.transmit())

def simulate_tdma():
    env = simpy.Environment()
    num_nodes = 4
    time_slots = [0, 1, 2, 3]  # Time slots assigned to each node
    transmission_latency = 20  # Example transmission latency (in time units)
    
    # Create a network graph
    G = nx.Graph()
    G.add_nodes_from(range(num_nodes))
    
    # Add edges to represent connectivity
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]  # Example edges (modify as needed)
    G.add_edges_from(edges)
    
    nodes = []
    for i in range(num_nodes):
        node = Node(env, i, time_slots[i], transmission_latency)
        env.process(node.run())
        nodes.append(node)
    
    env.run(until=100)  # Run the simulation for 10 time units

simulate_tdma()