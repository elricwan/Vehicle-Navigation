# -*- coding: utf-8 -*-
"""
Created on Fri Aug  3 11:06:59 2018

@author: Xiangpeng Wan
"""
import osmnx as ox
import networkx as nx
import requests
import matplotlib.cm as cm
import matplotlib.colors as colors
ox.config(use_cache=True, log_console=True)
ox.__version__

# get a graph for some city
# Simplification is normally done by OSMnx automatically under the hood
G = ox.graph_from_place('Hoboken, New Jersey', network_type = 'drive')
# this edge is wrong, it is equal
# We need to delete this edge
G.remove_edge(293301804, 293301804)
#ox.plot_graph(ox.project_graph(G))
fig, ax = ox.plot_graph(G)

# name the road with its order
Roads = list(G.edges())
check_list = {}
for i in range(len(Roads)):
    check_list[Roads[i]] = i + 1
    
# find the neighborhood road
def find_nei(road):
    Roads = list(G.edges())
    a = road[0]
    b = road[1]
    road_neibor = []
    for r in Roads:
        if r[0] == a or r[0] == b or r[1] == a or r[1] == b:
            road_neibor.append(r)
    return road_neibor

# Since the some speed information are str, some are digit, we need to conver all to digit
def str2num(speed):
    if type(speed) == list: # two data stored in the same road
        speed = speed[0]
    if len(speed) > 4:
        speed = speed[:2]
    return float(speed)

# find the maxspeed of the road
from numpy.random import RandomState
import statistics
def find_maxspeed(road):
    # some maxspeed stored in the map
    if 'maxspeed' in G.adj[road[0]][road[1]][0].keys():
        max_speed = str2num(G.adj[road[0]][road[1]][0]['maxspeed'])
    else:
        # get the mean of its neiborhood's street
        road_nei = find_nei(road)
        speed = [str2num(G.adj[r[0]][r[1]][0]['maxspeed']) for r in road_nei if 'maxspeed' in G.adj[r[0]][r[1]][0].keys()]
        if len(speed) > 1:
            max_speed = statistics.mean(speed)
        else:
            #exponential distrubution from 25 to 50.
            max_speed = 20 + RandomState().exponential(5)
    return max_speed

# most road has no nl infor, some most of them are generated with exponential distribution
def find_nl(road):
    # some nl stored in the map
    if 'lanes' in G.adj[road[0]][road[1]][0].keys():
        if len(G.adj[road[0]][road[1]][0]['lanes']) > 1: # some roads has different lanes per segment, choose the first one
            lanes = int(G.adj[road[0]][road[1]][0]['lanes'][0])
        else:
            lanes = int(G.adj[road[0]][road[1]][0]['lanes'])
    else:
        lanes = round(1 + RandomState().exponential(0.5))
        lanes = min(lanes,4)
    return lanes

import json
import math
d = 60 # this distance is working
# d = 100 # this distance too large.
# d = 10
n_intersection = len(G.nodes())
n_road = len(G.edges())
Region = {}
Region['Intersection'] = {}
for i in range(n_intersection):
    Intersect = list(G.nodes())[i]
    Region['Intersection'][Intersect] = G.node[Intersect]
Region['Segment'] = {}
for i in range(n_road):
    road = list(G.edges())[i]
    Region['Segment'][road] = {}
    # the length information is stored in graph
    distance = G.adj[road[0]][road[1]][0]['length']
    lanes = find_nl(road)
    max_speed = find_maxspeed(road)
        
    # number of subsegments for each road
    num_sub =  int(round(distance/d))
    if num_sub == 0:
        num_sub = 1
    Region['Segment'][road]['num_sub'] = num_sub
    # Number of lanes and maximum speed for the road
    Region['Segment'][road]['lanes'] = lanes
    Region['Segment'][road]['maxspeed'] = max_speed
    # name the road with its order
    Region['Segment'][road]['id'] = check_list[road]
    Region['Segment'][road]['subs'] = []
    # define x,y for each road
    x1 = Region['Intersection'][road[0]]['x']
    x2 = Region['Intersection'][road[1]]['x']
    y1 = Region['Intersection'][road[0]]['y']
    y2 = Region['Intersection'][road[1]]['y']
    Region['Segment'][road]['X'] = [x1,x2]
    Region['Segment'][road]['Y'] = [y1,y2]
    for j in range(num_sub):
        sub_distance = distance/num_sub
        # define x,y for each road subsegment
        a1 = Region['Intersection'][road[0]]['x'] + \
        j*(Region['Intersection'][road[1]]['x'] - Region['Intersection'][road[0]]['x'])/num_sub
        a2 = Region['Intersection'][road[0]]['x'] + \
        (j+1)*(Region['Intersection'][road[1]]['x'] - Region['Intersection'][road[0]]['x'])/num_sub # x coordiante for fist segment
        
        b1 = Region['Intersection'][road[0]]['y'] + \
        j*(Region['Intersection'][road[1]]['y'] - Region['Intersection'][road[0]]['y'])/num_sub
        b2 = Region['Intersection'][road[0]]['y'] + \
        (j+1)*(Region['Intersection'][road[1]]['y'] - Region['Intersection'][road[0]]['y'])/num_sub
        
        #Region['Segment'][road]['sub'+ str(j)] = {'x':[a1,a2] ,'y':[b1,b2], 'uturn': 0}
        Region['Segment'][road]['subs'] += [{'x':[a1,a2] ,'y':[b1,b2], 'distance':sub_distance,'uturn': 0}]
        
# Add In-Out to intersection
for i in range(n_intersection):
    # the name of the road is the order of the road
    Intersect = list(G.nodes())[i]
    Roads = list(G.edges())
    Out = [i+1 for i in range(n_road) if Roads[i][0] == Intersect]
    In = [i+1 for i in range(n_road) if Roads[i][1] == Intersect]
    
    Region['Intersection'][Intersect]['In'] = In
    Region['Intersection'][Intersect]['Out'] = Out
    
# Assign stop probability to each road by the number of 'In' road with different angle
# regroup the input road that share the same traffic light
# from some examples, I plan to make the error range as 0.2, means if two roads' k close enough(less than 1)
# then the two roads share the same traffic light
from collections import defaultdict
def group_line(Input):
    Roads = list(G.edges())
    k = {} # store the slop for every input road
    for r in Input:
        # minus one because the road index begin from 1, 1 represent 0 in Roads
        x_d = Region['Segment'][Roads[r-1]]['X'][0]-Region['Segment'][Roads[r-1]]['X'][1]
        y_d = Region['Segment'][Roads[r-1]]['Y'][0]-Region['Segment'][Roads[r-1]]['Y'][1]
        k[r] = x_d/y_d
    #regroup the input road
    group = defaultdict(list)
    group[0] = [Input[0]]
    for i in Input[1:]:
        belong_old_group = 0
        for j in range(len(group.keys())):
            content = group[j]
            if belong_old_group == 1:
                break
            for ii in range(len(content)):
                road = content[ii]
                if k[road]-1 < k[i] < k[road]+1:
                    group[j] += [i]
                    belong_old_group = 1
                    break          
        if belong_old_group == 0:
            new_group = len(group.keys())
            group[new_group] = [i]        
    return group

# Add logic traffic stop direction ti Intersection
# if there are two group of input data, the probability facing the traffic light is 0.5
# if there are there group of input data, the probability facing the traffic light is 1/3
for i in range(n_intersection):
    Intersect = list(G.nodes())[i]
    Input = Region['Intersection'][Intersect]['In']
    Output = Region['Intersection'][Intersect]['Out']
    if len(Input) > 0:
        n_input = len(group_line(Input)) # calculate the number of input with different angle
        n_output = len(Output)
        Roads = list(G.edges())
        for j in range(len(Input)):
        # for which input, there would be most 2 outputs with different traffic light (left,straight)
            p = 1 - 1/(n_input*min(2,n_output))
            r = Roads[Input[j]-1]
            Region['Segment'][r]['probability'] = p
            
# Find the corresponding uturn road
import numpy as np
To_road = list(Region['Segment'].keys())
for road in To_road:
    if G.adj[road[0]][road[1]][0]['oneway'] != True:
        i = road[::-1]
        for road_seg in range(Region['Segment'][road]['num_sub']-1):
            target_x = Region['Segment'][road]['subs'][road_seg]['x'][::-1]
            target_y = Region['Segment'][road]['subs'][road_seg]['y'][::-1]
            dis = []
            for j in range(Region['Segment'][i]['num_sub']):  # we do not consider the segment into intersection
                x1 = Region['Segment'][i]['subs'][j]['x'][0]
                x2 = Region['Segment'][i]['subs'][j]['x'][1]
                y1 = Region['Segment'][i]['subs'][j]['y'][0]
                y2 = Region['Segment'][i]['subs'][j]['y'][1]
                # find the minimum dist from the street's end to the uturn's start
                dis.append((target_x[0] - x1)**2 + (target_y[0] - y1)**2)
            if len(dis) > 0:
                ind = np.argmin(dis)
                ind = int(ind)
                # Transform the road to its name, et, its order in the list.
                # add one because in matlab everything go from 1
                raod_name = check_list[i]
                Region['Segment'][road]['subs'][road_seg]['uturn'] = [raod_name,ind+1]  

# add event
import random
for i in range(n_road):
    r = list(G.edges())[i]
    Event = [0] * 5000
    if random.random() < 0.1: # 10% probability to have events
        start = random.randint(1,n_road-300) 
        duration = random.randint(30,500)
        end = min(500,start+duration)
        Event[start:end] = [1]*(end-start)
        Region['Segment'][r]['Event'] = Event
    else:
        Region['Segment'][r]['Event'] = Event
        
# Output json file
# First I need to transform every key to str
Intersection = Region['Intersection']
with open('Intercept.json', 'w') as outfile:
    json.dump(Intersection, outfile)
    
# Output json file
# First I need to transform every key to str
Segment = dict([(str(a).replace(' ','')[1:-1],b) for (a,b) in Region['Segment'].items()])
Intersection = Region['Intersection']
with open('Segment.json', 'w') as outfile:  
    json.dump(Segment, outfile)