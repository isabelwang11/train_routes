import sys
import time
from heapq import heappop, heappush

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

def train_routes_dijkstra(city1, city2, nodes, edges, city_id):
    id1 = city_id[city1]
    lat1, long1 = nodes[id1]
    id2 = city_id[city2]
    lat2, long2 = nodes[id2]
    closed = set()
    fringe = [(calcd(lat1, long1, lat2, long2), 0, id1)]
    while len(fringe) != 0:
        heuristic, distance, node_id = heappop(fringe)
        if node_id == id2:
            return distance
        if node_id not in closed:
            closed.add(node_id)
            for c in edges[node_id]:
                c_gcd, c_id = c
                latc, longc = nodes[c_id]
                heappush(fringe, (c_gcd + distance, c_gcd + distance, c_id))
    return None

def train_routes_Astar(city1, city2, nodes, edges, city_id):
    id1 = city_id[city1]
    lat1, long1 = nodes[id1]
    id2 = city_id[city2]
    lat2, long2 = nodes[id2]
    closed = set()
    fringe = [(calcd(lat1, long1, lat2, long2), 0, id1)]
    while len(fringe) != 0:
        heuristic, distance, node_id = heappop(fringe)
        if node_id == id2:
            return distance
        if node_id not in closed:
            closed.add(node_id)
            for c in edges[node_id]:
                c_gcd, c_id = c
                latc, longc = nodes[c_id]
                heappush(fringe, (c_gcd+distance+calcd(latc, longc, lat2, long2), c_gcd+distance, c_id))
    return None

nodes = {}
with open("rrNodes.txt") as f_nodes:
    for line in f_nodes:
        id = line.split()[0]
        lat = line.split()[1]
        long = line.split()[2]
        nodes[id] = (lat, long)

start = time.perf_counter()
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
end = time.perf_counter()
print("Time to create data structure: %s seconds" % str(end-start))

city_id = {}
with open("rrNodeCity.txt") as f_cities:
    for line in f_cities:
        id = line.split()[0]
        city = line[8:-1]
        city_id[city] = id

city1 = sys.argv[1]
city2 = sys.argv[2]
start_D = time.perf_counter()
trD = train_routes_dijkstra(city1, city2, nodes, edges, city_id)
end_D = time.perf_counter()
print("%s to %s with Dijkstra: %s in %s seconds" % (city1, city2, str(trD), str(end_D-start_D)))
start_Astar = time.perf_counter()
trA = train_routes_Astar(city1, city2, nodes, edges, city_id)
end_Astar = time.perf_counter()
print("%s to %s with A*: %s in %s seconds" % (city1, city2, str(trA), str(end_Astar-start_Astar)))