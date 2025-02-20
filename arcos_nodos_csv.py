import osmnx as ox
import time
from shapely.geometry import Polygon
import os
import csv
import numpy as np

def save_graph_shapefile_directional(name_file, G, filepath=None, encoding="utf-8"):
    # default filepath if none was provided
    if filepath is None:
        filepath = os.path.join(ox.settings.data_folder, "graph_shapefile")

    # if save folder does not already exist, create it (shapefiles
    # get saved as set of files)
    if not filepath == "" and not os.path.exists(filepath):
        os.makedirs(filepath)
    filepath_nodes = os.path.join(filepath, "nodes.shp")
    filepath_edges = os.path.join(filepath, "edges.shp")


    # convert undirected graph to gdfs and stringify non-numeric columns
    gdf_nodes, gdf_edges = ox.utils_graph.graph_to_gdfs(G)
    gdf_nodes = ox.io._stringify_nonnumeric_cols(gdf_nodes)
    gdf_edges = ox.io._stringify_nonnumeric_cols(gdf_edges)
    # We need an unique ID for each edge
    # filtramos calles
    # gdf_edges = gdf_edges[gdf_edges["highway"]!='living_street']
    # gdf_edges = gdf_edges[gdf_edges["highway"]!='residential']
    # gdf_edges = gdf_edges[gdf_edges["highway"]!="['living_street', 'residential']"]
    # gdf_edges = gdf_edges[gdf_edges["highway"]!="['residential', 'living_street']"]
    # ids = [i + 1 for i in range(len(gdf_edges.index))]
    # gdf_edges["id"] = ids
    gdf_edges["fid"] = np.arange(0, gdf_edges.shape[0], dtype='int')

    # save the nodes and edges as separate ESRI shapefiles
    gdf_nodes.to_file(filepath_nodes, encoding=encoding)
    gdf_edges.to_file(filepath_edges, encoding=encoding)

    # a = gdf_nodes.to_json()
    # b = gdf_edges.to_json()
    #
    # print(gdf_nodes.columns)
    # print(gdf_nodes.head())
    # print(gdf_edges.columns)
    # print(gdf_edges.head())

    # archivo nodo to csv
    osmid = [i for i in gdf_nodes.index]
    y = list(gdf_nodes['y'])
    x = list(gdf_nodes['x'])
    street_count = list(gdf_nodes['street_count'])
    highway = list(gdf_nodes['highway'])
    geometry = list(gdf_nodes['geometry'])

    with open(os.path.join("osm/{}/{}_nodos_completo.csv".format(name_file, name_file)), "w", newline="",
              encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(["osmid", "y", "x", "street_count", "highway", "geometry"])

        for i in range(len(list(gdf_nodes['y']))):
            writer.writerow([osmid[i], y[i], x[i], street_count[i], highway[i], geometry[i]])

    # archivo arco to csv
    fid = np.arange(0, gdf_edges.shape[0], dtype='int')
    osmid = list(gdf_edges['osmid'])
    u = [u for u, v, key in gdf_edges.index]
    v = [v for u, v, key in gdf_edges.index]
    key = [key for u, v, key in gdf_edges.index]
    highway = list(gdf_edges['highway'])
    oneway = list(gdf_edges['oneway'])
    length = list(gdf_edges['length'])
    geometry = list(gdf_edges['geometry'])
    name = list(gdf_edges['name'])
    lanes = list(gdf_edges['lanes'])
    maxspeed = list(gdf_edges['maxspeed'])

    with open(os.path.join("osm/{}/{}_arcos_completo.csv".format(name_file, name_file)), "w", newline="",
              encoding="utf-8") as f:
        writer = csv.writer(f, delimiter=",")
        writer.writerow(
            ["fid", "osmid", "osmid_nodo_origen", "osmid_nodo_destino", "key", "highway", "oneway", "length",
             "geometry",
             "name", "lanes", "maxspeed"])

        for i in range(len(list(gdf_edges['osmid']))):
            writer.writerow(
                [fid[i], osmid[i], u[i], v[i], key[i], highway[i], oneway[i], length[i], geometry[i], name[i], lanes[i],
                 maxspeed[i]])


print("osmnx version", ox.__version__)
# Download by a bounding box
# calama_box =  -69.010593,-22.528548,-68.832065,-22.296219
# santiago_box = -71.233366,-33.986168,-70.146269,-32.987462
# x1, y1, x2, y2 = -71.233366, -33.986168, -70.146269, -32.987462
x1, y1, x2, y2 = 13.1127, 52.5235, 13.2132, 52.5520
boundary_polygon = Polygon([(x1, y1), (x2, y1), (x2, y2), (x1, y2)])
G = ox.graph_from_polygon(boundary_polygon, network_type='drive')

start_time = time.time()
save_graph_shapefile_directional('berlin', G, filepath='osm/berlin')
print("--- %s seconds ---" % (time.time() - start_time))
