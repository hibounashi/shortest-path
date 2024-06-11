import tkinter as tk
from tkinter import ttk, messagebox
import networkx as nx
import matplotlib.pyplot as plt
import heapq
import osmnx as ox
import os
import folium

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

def plot_shortest_path(graph, shortest_path):
    nodes_coordinates = {node: (float(graph.nodes[node]['y']), float(graph.nodes[node]['x'])) for node in graph.nodes}
    
    G_map = folium.Map(location=[nodes_coordinates[shortest_path[0]][0], nodes_coordinates[shortest_path[0]][1]], zoom_start=14)
    
    for node in shortest_path:
        folium.Marker(location=[nodes_coordinates[node][0], nodes_coordinates[node][1]], popup=str(node), icon=folium.Icon(color='red')).add_to(G_map)
    
    folium.PolyLine(locations=[nodes_coordinates[node] for node in shortest_path], color="red").add_to(G_map)
    
    return G_map

def find_shortest_path(city):
    start_node = start_combo.get()
    goal_node = goal_combo.get()
    
    shortest_path = astar(graph[city], start_node, goal_node)

    if shortest_path:
        map_with_path = plot_shortest_path(graph[city], shortest_path)
        map_with_path.save("shortest_path.html")
        os.system("shortest_path.html")
    else:
        messagebox.showinfo("No Path Found", "No path found between the given nodes.")

def city_selected(event):
    selected_city = city_combo.get()
    start_combo['values'] = list(graph[selected_city].nodes)
    goal_combo['values'] = list(graph[selected_city].nodes)

osm_files = {
    'Algiers': r'.\algiers_map.osm',
    'Bejaia': r'.\bejaia_map.osm'
}

graphml_files = {
    'Algiers': r'.\algiers_map.graphml',
    'Bejaia': r'.\bejaia_map.graphml'
}

graph = {}

# Convert OSM file to GraphML if not already done
for city, osm_file_path in osm_files.items():
    if not os.path.exists(graphml_files[city]):
        graph[city] = ox.graph_from_xml(osm_file_path)
        ox.save_graphml(graph[city], graphml_files[city])
    else:
        graph[city] = nx.read_graphml(graphml_files[city])

# Create UI
root = tk.Tk()
root.title("Shortest Path Finder")
root.geometry("800x600")

style = ttk.Style(root)
style.configure('TButton', font=('calibri', 12, 'bold'), foreground='black', background='lightblue')
style.configure('TLabel', font=('calibri', 12), foreground='black')
style.configure('TCombobox', font=('calibri', 12), foreground='black', background='lightgrey')

frame = ttk.Frame(root)
frame.grid(padx=20, pady=20)

title_label = ttk.Label(frame, text="Choose The Appropriate Value ", font=('calibri', 16, 'bold'))
title_label.grid(row=0, column=0, columnspan=2, pady=20)

city_label = ttk.Label(frame, text="Select City:")
city_label.grid(row=1, column=0, padx=10, pady=10, sticky="e")

cities = list(osm_files.keys())
city_combo = ttk.Combobox(frame, values=cities, state="readonly")
city_combo.grid(row=1, column=1, padx=10, pady=10)
city_combo.bind("<<ComboboxSelected>>", city_selected)

start_label = ttk.Label(frame, text="from:")
start_label.grid(row=2, column=0, padx=10, pady=10, sticky="e")

start_combo = ttk.Combobox(frame, values=[], state="readonly")
start_combo.grid(row=2, column=1, padx=10, pady=10)

goal_label = ttk.Label(frame, text="to:")
goal_label.grid(row=3, column=0, padx=10, pady=10, sticky="e")

goal_combo = ttk.Combobox(frame, values=[], state="readonly")
goal_combo.grid(row=3, column=1, padx=10, pady=10)

find_path_button = ttk.Button(frame, text="Search", command=lambda: find_shortest_path(city_combo.get()), style='TButton')
find_path_button.grid(row=4, column=0, columnspan=2, padx=10, pady=10, sticky="we")

frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

root.mainloop()
