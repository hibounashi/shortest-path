import tkinter as tk
from tkinter import ttk, messagebox
import networkx as nx
import matplotlib.pyplot as plt
import heapq
import osmnx as ox
import os

def astar(graph, start, goal):
    def heuristic(node):
        x1, y1 = float(graph.nodes[start]['x']), float(graph.nodes[start]['y'])
        x2, y2 = float(graph.nodes[node]['x']), float(graph.nodes[node]['y'])
        return ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    
    open_set = [(0, start)]  
    came_from = {}  
    g_score = {node: float('inf') for node in graph.nodes}  
    g_score[start] = 0
    
    while open_set:
        current_cost, current_node = heapq.heappop(open_set)
        if current_node == goal:
            path = []
            while current_node in came_from:
                path.append(current_node)
                current_node = came_from[current_node]
            path.append(start)
            path.reverse()
            return path
        
        for u, v, key, edge_data in graph.edges(keys=True, data=True):
            if u == current_node:
                if 'length' in edge_data:
                    tentative_g_score = g_score[current_node] + float(edge_data['length'])
                    if tentative_g_score < g_score[v]:
                        came_from[v] = current_node
                        g_score[v] = tentative_g_score
                        f_score = tentative_g_score + heuristic(v)
                        heapq.heappush(open_set, (f_score, v))
                else:
                    raise ValueError("Edge missing 'length' attribute")
    
    return None

def find_shortest_path():
    start_node = start_combo.get()
    goal_node = goal_combo.get()
    
    shortest_path = astar(graph, start_node, goal_node)

    if shortest_path:
        shortest_path_edges = [(shortest_path[i], shortest_path[i+1]) for i in range(len(shortest_path)-1)]
        plt.figure(figsize=(10, 8))
        pos = nx.spring_layout(graph)  
        nx.draw(graph, pos, with_labels=True, node_size=300, node_color='skyblue')
        nx.draw_networkx_edges(graph, pos, edgelist=shortest_path_edges, width=4, edge_color='pink', arrows=True)  
        plt.title(f"Shortest Path from {start_node} to {goal_node}")
        plt.show()
    else:
        messagebox.showinfo("No Path Found", "No path found between the given nodes.")

osm_file_path = r'.\map.osm'
graphml_file_path = r'.\map.graphml'

# Convert OSM file to GraphML if not already done
if not os.path.exists(graphml_file_path):
    graph = ox.graph_from_xml(osm_file_path)
    ox.save_graphml(graph, graphml_file_path)

# Load the graph from GraphML file
graph = nx.read_graphml(graphml_file_path)

# Create UI
root = tk.Tk()
root.title("Shortest Path Finder")
root.geometry("800x600")  # Larger window size

style = ttk.Style(root)
style.configure('TButton', font=('calibri', 12, 'bold'), foreground='black', background='lightblue')  # Button styling
style.configure('TLabel', font=('calibri', 12), foreground='black')  # Label styling
style.configure('TCombobox', font=('calibri', 12), foreground='black', background='lightgrey')  # Combobox styling

frame = ttk.Frame(root)
frame.grid(padx=20, pady=20)  # Padding increased

title_label = ttk.Label(frame, text="Choose Your Position and Destination", font=('calibri', 16, 'bold'))
title_label.grid(row=0, column=0, columnspan=2, pady=20)  # Title label padding increased

start_label = ttk.Label(frame, text="Start Node:")
start_label.grid(row=1, column=0, padx=10, pady=10, sticky="e")

start_combo = ttk.Combobox(frame, values=list(graph.nodes), state="readonly")
start_combo.grid(row=1, column=1, padx=10, pady=10)

goal_label = ttk.Label(frame, text="Goal Node:")
goal_label.grid(row=2, column=0, padx=10, pady=10, sticky="e")

goal_combo = ttk.Combobox(frame, values=list(graph.nodes), state="readonly")
goal_combo.grid(row=2, column=1, padx=10, pady=10)

find_path_button = ttk.Button(frame, text="Search", command=find_shortest_path, style='TButton')
find_path_button.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="we")

frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

root.mainloop()
