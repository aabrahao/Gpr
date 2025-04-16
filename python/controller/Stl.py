import numpy as np
from stl import mesh
import pyvista as pv

def filename(path):
    return path + '.stl'

def load(path):
    return mesh.Mesh.from_file( filename(path) )

def vertices(mesh):
    vertices = np.concatenate((mesh.v0, mesh.v1, mesh.v2))
    vertices = np.unique(vertices, axis=0)
    x = vertices[:, 0]
    y = vertices[:, 1]
    z = vertices[:, 2]
    return x,y,z
