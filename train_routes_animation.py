import sys
import time
from heapq import heappop, heappush
from tkinter import *

#
# Torbert, 22 Sept 2014
# White (ed), 5 Oct 2016
#
from math import pi , acos , sin , cos
#
def calcd(y1,x1, y2,x2):
   #
   # y1 = lat1, x1 = long1
   # y2 = lat2, x2 = long2
   # all assumed to be in decimal degrees

   # if (and only if) the input is strings
   # use the following conversions

   y1  = float(y1)
   x1  = float(x1)
   y2  = float(y2)
   x2  = float(x2)
   #
   R   = 3958.76 # miles = 6371 km
   #
   y1 *= pi/180.0
   x1 *= pi/180.0
   y2 *= pi/180.0
   x2 *= pi/180.0
   #
   # approximate great circle distance with law of cosines
   #
   return acos( sin(y1)*sin(y2) + cos(y1)*cos(y2)*cos(x2-x1) ) * R
   #
#
# end of file
#

def train_routes_dijkstra(city1, city2, nodes, edges, city_id, max_lat, min_lat, min_long):
    id1 = city_id[city1]
    lat1, long1 = nodes[id1]
    id2 = city_id[city2]
    lat2, long2 = nodes[id2]
    closed = set()
    fringe = [(calcd(lat1, long1, lat2, long2), 0, id1, [])]
    ct = 0
    while len(fringe) != 0:
        heuristic, distance, node_id, path = heappop(fringe)
        if node_id == id2:
            return path
        if node_id not in closed:
            closed.add(node_id)
            lat, long = nodes[node_id]
            for c in edges[node_id]:
                c_gcd, c_id = c
                latc, longc = nodes[c_id]
                coordlong1 = abs(long - min_long)
                coordlat1 = abs(lat - min_lat)
                coordlong2 = abs(longc - min_long)
                coordlat2 = abs(latc - min_lat)
                l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1) - 120, 8 * (coordlong2), 8 * (max_lat - coordlat2) - 120, fill="cyan")
                ct += 1
                updated_path = path.copy()
                updated_path.append((c_id))
                heappush(fringe, (c_gcd + distance, c_gcd + distance, c_id, updated_path))
                if ct%int(10*speed_scale.get())==0:
                    c_d.update()
    return None

def train_routes_Astar(city1, city2, nodes, edges, city_id, max_lat, min_lat, min_long):
    id1 = city_id[city1]
    lat1, long1 = nodes[id1]
    id2 = city_id[city2]
    lat2, long2 = nodes[id2]
    closed = set()
    fringe = [(calcd(lat1, long1, lat2, long2), 0, id1, [])]
    ct = 0
    while len(fringe) != 0:
        heuristic, distance, node_id, path = heappop(fringe)
        if node_id == id2:
            return path
        if node_id not in closed:
            closed.add(node_id)
            lat, long = nodes[node_id]
            for c in edges[node_id]:
                c_gcd, c_id = c
                latc, longc = nodes[c_id]
                coordlong1 = abs(long - min_long)
                coordlat1 = abs(lat - min_lat)
                coordlong2 = abs(longc - min_long)
                coordlat2 = abs(latc - min_lat)
                l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1)-120, 8 * (coordlong2), 8 * (max_lat-coordlat2)-120, fill="purple")
                ct += 1
                updated_path = path.copy()
                updated_path.append((c_id))
                heappush(fringe, (c_gcd+distance+calcd(latc, longc, lat2, long2), c_gcd+distance, c_id, updated_path))
                if ct%int(10*speed_scale.get())==0:
                    c_d.update()
    return None

def train_routes_DFS(city1, city2, nodes, edges, city_id, max_lat, min_lat, min_long):
    id1 = city_id[city1]
    id2 = city_id[city2]
    fringe = [(id1, 0, [])]
    visited = set()
    visited.add(id1)
    if id1 == id2:
        return 0
    ct = 0
    while len(fringe) != 0:
        node_id, distance, path = fringe.pop()
        if node_id == id2:
            return path
        lat, long = nodes[node_id]
        for c in edges[node_id]:
            if c not in visited:
                c_gcd, c_id = c
                latc, longc = nodes[c_id]
                updated_path = path.copy()
                updated_path.append((c_id))
                fringe.append((c_id, c_gcd+distance, updated_path))
                visited.add(c)
                coordlong1 = abs(long - min_long)
                coordlat1 = abs(lat - min_lat)
                coordlong2 = abs(longc - min_long)
                coordlat2 = abs(latc - min_lat)
                l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1) - 120, 8 * (coordlong2), 8 * (max_lat - coordlat2) - 120, fill="green")
                ct += 1
                if ct%int(speed_scale.get())==0:
                    c_d.update()
    return None

def train_routes_IDDFS(city1, city2, nodes, edges, city_id, max_lat, min_lat, min_long):
    k = 1
    id1 = city_id[city1]
    id2 = city_id[city2]
    while True:
        distance = K_DFS(id1, id2, nodes, edges, city_id, max_lat, min_lat, min_long, k)
        if distance != None:
            return distance
        k += 1
    return None

def K_DFS(id1, id2, nodes, edges, city_id, max_lat, min_lat, min_long, k):
    fringe = [(id1, 0, {id1}, 0)]
    while len(fringe) != 0:
        ct = 0
        node_id, depth, ancestors, distance = fringe.pop()
        if node_id == id2:
            return distance
        lat, long = nodes[node_id]
        if depth < k:
            for c in edges[node_id]:
                c_gcd, c_id = c
                if c_id not in ancestors:
                    latc, longc = nodes[c_id]
                    fringe.append((c_id, depth + 1, ancestors.union({c_id}), c_gcd+distance))
                    coordlong1 = abs(long - min_long)
                    coordlat1 = abs(lat - min_lat)
                    coordlong2 = abs(longc - min_long)
                    coordlat2 = abs(latc - min_lat)
                    l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1) - 120, 8 * (coordlong2), 8 * (max_lat - coordlat2) - 120, fill="blue")
                    ct += 1
                if ct%(1000*int(speed_scale.get()))==0: # fix speed
                    c_d.update()
    return None

def run_alg():
    alg_name = tkvar3.get()
    reset_map(c_d)
    if alg_name == "Dijkstra":
        path = train_routes_dijkstra(tkvar1.get(), tkvar2.get(), nodes, edges, city_id, max_lat, min_lat, min_long)
    elif alg_name == "A*":
        path = train_routes_Astar(tkvar1.get(), tkvar2.get(), nodes, edges, city_id, max_lat, min_lat, min_long)
    elif alg_name == "DFS":
        path = train_routes_DFS(tkvar1.get(), tkvar2.get(), nodes, edges, city_id, max_lat, min_lat, min_long)
    elif alg_name == "ID-DFS":
        train_routes_IDDFS(tkvar1.get(), tkvar2.get(), nodes, edges, city_id, max_lat, min_lat, min_long)
        path = []
    else:
        print("Error: Invalid algorithm selected")
    if alg_name == "Dijkstra" or alg_name == "A*" or alg_name == "DFS":
        prev_node_id = path[0]
        latprev, longprev = nodes[prev_node_id]
        for node_id in path:
            lat, long = nodes[node_id]
            coordlong1 = abs(longprev - min_long)
            coordlat1 = abs(latprev - min_lat)
            coordlong2 = abs(long - min_long)
            coordlat2 = abs(lat - min_lat)
            l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1) - 120, 8 * (coordlong2), 8 * (max_lat - coordlat2) - 120, fill="pink", width=3)
            prev_node_id = node_id
            latprev, longprev = lat, long

def reset_map(c_d):
    c_d.delete('all')
    with open("rrEdges.txt") as f_edges:
        for line in f_edges:
            id1 = line.split()[0]
            id2 = line.split()[1]
            lat1, long1 = nodes[id1]
            lat2, long2 = nodes[id2]
            coordlong1 = abs(long1 - min_long)
            coordlat1 = abs(lat1 - min_lat)
            coordlong2 = abs(long2 - min_long)
            coordlat2 = abs(lat2 - min_lat)
            l = c_d.create_line(8 * (coordlong1), 8 * (max_lat - coordlat1) - 120, 8 * (coordlong2), 8 * (max_lat - coordlat2) - 120, fill="white")
    c_d.pack()

# generate nodes
nodes = {}
min_lat, max_lat, min_long, max_long = 1000, 0, 0, -1000
with open("rrNodes.txt") as f_nodes:
    for line in f_nodes:
        id = line.split()[0]
        lat = float(line.split()[1])
        long = float(line.split()[2])
        if lat < min_lat:
            min_lat = lat
        elif lat > max_lat:
            max_lat = lat
        if long < min_long:
            min_long = long
        elif long > max_long:
            max_long = long
        nodes[id] = (lat, long)

# create window and frames
master = Tk() # window
h = 8*(max_lat - min_lat)
w = 8*(max_long - min_long)
master.geometry("%sx%s" % (str(int(w)+10), str(int(h)+100))) #figure out size
ctrlFrame = Frame(master)
ctrlFrame.grid(row=0)
drawFrame = Frame(master)
drawFrame.grid(row=1)

# canvas
c_d = Canvas(drawFrame, bg="black", height=h, width=w)
c_d.pack()

# generating map
edges = {}
with open("rrEdges.txt") as f_edges:
    for line in f_edges:
        id1 = line.split()[0]
        id2 = line.split()[1]
        lat1, long1 = nodes[id1]
        lat2, long2 = nodes[id2]
        gcd = calcd(lat1, long1, lat2, long2)
        if id1 not in edges:
            edges[id1] = [(gcd, id2)]
        else:
            edges[id1].append((gcd, id2))
        if id2 not in edges:
            edges[id2] = [(gcd, id1)]
        else:
            edges[id2].append((gcd, id1))
    reset_map(c_d)

# generate cities w/ names
city_id = {}
with open("rrNodeCity.txt") as f_cities:
    for line in f_cities:
        id = line.split()[0]
        city = line[8:-1]
        city_id[city] = id

# controls
city1_label = Label(ctrlFrame, text="City #1: ", fg="red")
city2_label = Label(ctrlFrame, text="City #2: ", fg="blue")
algorithm_label = Label(ctrlFrame, text="Algorithm: ", fg="green")
speed_label = Label(ctrlFrame, text="Speed: ", fg="purple")
tkvar1 = StringVar(ctrlFrame)
tkvar1.set('Select a city')
city1_menu = OptionMenu(ctrlFrame, tkvar1, *city_id.keys())
tkvar2 = StringVar(ctrlFrame)
tkvar2.set('Select a city')
city2_menu = OptionMenu(ctrlFrame, tkvar2, *city_id.keys())
tkvar3 = StringVar(ctrlFrame)
tkvar3.set('Select an algorithm')
alg_op = {"Dijkstra", "A*", "DFS", "ID-DFS"}
algorithm_menu = OptionMenu(ctrlFrame, tkvar3, *alg_op)
speed_scale = Scale(ctrlFrame, from_=1, to=100, orient=HORIZONTAL)
run_button = Button(ctrlFrame, text="Run", command=run_alg) # change command
quit_button = Button(ctrlFrame, text="Quit", command=quit)
city1_label.grid(row=0)
city2_label.grid(row=0, column=1)
algorithm_label.grid(row=0, column=2)
speed_label.grid(row=0, column=3)
city1_menu.grid(row=1)
city2_menu.grid(row=1, column=1)
algorithm_menu.grid(row=1, column=2)
speed_scale.grid(row=1, column=3)
run_button.grid(row=2, column=1)
quit_button.grid(row=2, column=2)

master.mainloop()

# def printMessage():
#     print("hello world")
#
# root = Tk()
#
# city1_label = Label(root, text="Depart from: ", bg="red")
# tkvar = StringVar(root)
# choices = {'Select an option', 'Pizza','Lasagne','Fries','Fish','Potatoes'}
# tkvar.set('Select an option')
# city1_menu = OptionMenu(root, tkvar, *choices)
# Label(root, text="Choose a dish").grid(row = 1, column = 1)
# city2_label = Label(root, text="Arrive in: ", bg="blue", fg="white")
# city1_label.grid(row=0, sticky=E)
# city2_label.grid(row=1, sticky=E)
# city1_menu.grid(row=0, column=1)
# city2_menu.grid(row=1, column=1)
#
# run_button = Button(text="Run", fg="blue", command=printMessage)
# run_button.grid(row=2)
#
# quit_button = Button(text="Quit", command=quit)
# quit_button.grid(row=2, column=1)
#
# root.mainloop()