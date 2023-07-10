import simpy
import networkx as nx

class Node:
    def __init__(self, env, node_id, event_threshold, transmission_latency):
        self.env = env
        self.node_id = node_id
        self.event_threshold = event_threshold
        self.transmission_latency = transmission_latency
        self.channel = simpy.Resource(env, capacity=1)
        
    def detect_event(self):
        # Simulated event detection process
        if self.env.now % self.event_threshold == 0:
            return True
        else:
            return False
    
    def transmit(self):
        print(f"Node {self.node_id} transmitting at time {self.env.now}")
        yield self.env.timeout(self.transmission_latency)  # Transmission latency
    
    def run(self):
        while True:
            if self.detect_event():
                with self.channel.request() as req:
                    yield req
                    yield self.env.process(self.transmit())
            else:
                yield self.env.timeout(1)  # Wait for next time slot

def simulate_event_trigger():
    env = simpy.Environment()
    num_nodes = 4
    event_thresholds = [2, 3, 4, 5]  # Event detection thresholds for each node
    transmission_latency = 20  # Example transmission latency (in time units)
    
    # Create a network graph
    G = nx.Graph()
    G.add_nodes_from(range(num_nodes))
    
    # Add edges to represent connectivity
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]  # Example edges (modify as needed)
    G.add_edges_from(edges)
    
    nodes = []
    for i in range(num_nodes):
        node = Node(env, i, event_thresholds[i], transmission_latency)
        env.process(node.run())
        nodes.append(node)
    
    env.run(until=100)  # Run the simulation for 10 time units

simulate_event_trigger()