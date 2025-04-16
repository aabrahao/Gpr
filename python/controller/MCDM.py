import Hydrology as hyd
import Geotiff as gt
import DEM as dm
import Visualization as vz
import numpy as np
import Stl as st
from scipy import stats as sts
import matplotlib.pylab as plt

import Numeric as nm
import Quantification as qt

g_path = 'data/mcdm/mars'
g_view = (45,5)
g_save = True

class Data:
    RAW = 1
    MASK = 2
    PREDICTION = 3
    UNCERTAINTY = 4

def setProject(path, save, view = None):
    global g_path, g_view, g_save
    g_path = path
    g_view = view
    g_save = save

def plot(x,y,Z,C,title=''):
    if type(C) is list:
        i = 1
        for c in C:
            if title:
                label = title + f'{i}'
            vz.plotMesh(x,y,Z,nm.nanify(c,0.0),title=label,view=g_view)
            i = i + 1
    else:
        vz.plotMesh(x,y,Z,nm.nanify(C,0.0),title=title,view=g_view) 

def open(name=''):
    dataset = gt.open(g_path + name)
    x,y,Z = gt.dem(dataset)
    Z = nm.clean(Z)
    return x,y,Z

def dem(name):
    x,y,Z = open(name)
    return Z

def load(criteria):
    maps = []
    for name in criteria:
        map = dem(name)
        maps.append(map)
    return maps

def filter(data, type):
    if type == Data.MASK:
        return nm.mask(data, 0.0)
    return data

def rank(criteria, maps):
    rankings = []
    for map, (name, parameters) in zip (maps, criteria.items()):
        weight, treshold, type = parameters
        ranking = nm.trim(map, treshold)
        ranking = filter(ranking, type)
        ranking = nm.normalize(ranking)
        rankings.append(ranking)
    return rankings

def judge(criteria, rankings):
    priority = 0
    for ranking, (name, parameters) in zip (rankings, criteria.items()):
        weight, treshold, type = parameters
        priority = priority + weight*ranking
    priority = nm.normalize(priority)
    return priority

def emphasize(priority, levels):
    if nm.scalar(levels):
        return nm.trim(priority, levels) 
    maps = []
    for level in levels:
        map = nm.trim(priority, level)
        maps.append(map)
    return maps