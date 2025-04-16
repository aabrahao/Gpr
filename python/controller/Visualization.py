import Geotiff as gt
import pyvista as pv
import numpy as np
import DEM as dm
import Stl as stl
import matplotlib.pyplot as plt
import os

g_save_image = False
g_window_background = 'white'
g_colormap = 'jet'
g_mesh_color = 'lightgray'
g_point_size = 8

def saveImages(save):
    global g_save_image
    g_save_image = save
    
def plotDem(database,title='',edges=False,view=None):
    x,y,z = gt.dem(database)
    plotMesh(x,y,z,z,title,edges,view)

def plotMesh(x,y,Z,C=None,title='',edges=False,view=None):
    dm.checkIndexing(x,y,Z)
    X, Y = dm.grid(x,y)
    grid = pv.StructuredGrid(*(X, Y), Z)
    plotter = pv.Plotter()
    if C is None:
        plotter.add_mesh(grid, 
                         color=g_mesh_color,
                         show_edges=edges)
    else:
        plotter.add_mesh( 
            grid, 
            scalars=colormap(C),
            scalar_bar_args = scalarbar(title),
            show_edges=edges,
            cmap=g_colormap)
    decorate(plotter,title,view,C)
    plotter.show(title=title,auto_close=False)

def plotPoints(x,y,z,c=None,title='',view=None):
    C = c.copy()
    if C is None:
        C = z
    point_cloud = pv.PolyData(np.column_stack((x, y, z)))
    plotter = pv.Plotter()
    plotter.add_mesh(point_cloud, 
                     scalars=colormap(C), 
                     cmap=g_colormap,
                     scalar_bar_args = scalarbar(title),
                     point_size=g_point_size,
                     render_points_as_spheres=True)
    decorate(plotter,title,view,C)
    plotter.show(title=title,auto_close=False)

def plotStl(path,view=None,edges=False,title=''):
    mesh = pv.read( stl.filename(path) )
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color=g_mesh_color, show_edges=edges)
    changeBackground(plotter)
    changeView(plotter,view)
    savePdf(plotter,title)
    plotter.show(stl.filename(path),auto_close=False)

def plotHystogram(v,n=25):
    x = v.copy()
    x = x.flatten()
    print(f'Mean: {np.nanmean(x)}')
    print(f'Std: {np.nanmean(x)}')
    print(f'Std: {np.nanmean(x)}')
    print(f'Zscore: {zscore(x)}')
    print(f'Max: {np.nanmax(x)}')
    print(f'Max: {np.nanmin(x)}')
    plt.hist(x, bins=n, 
             color='skyblue', 
             edgecolor='black', 
             alpha=0.7)
    plt.show(auto_close=False)

# Stats ##############################################################

def zscore(data):
    return (data - np.nanmean(data)) / np.nanstd(data)

# Decoration ##############################################################

def filename(path):
    return os.path.splitext(os.path.basename(path))[0]

def scalarbar(title):
    args = {'title' :  filename(title),
            #'mapper' : None,
            #'n_labels' : 5,
            #'italic' : False,
            #'bold' : False,
            #'title_font_size' : None,
            #'label_font_size' : None,
            #'color' : None,
            #'font_family' : None,
            #'shadow' : False,
            #'width' : None,
            #'height' : None,
            #'position_x' : None,
            #'position_y' : None,
            'vertical' : True
            #'interactive' : None,
            #fmt' : None,
            #'use_opacity' : True,
            #'outline' : False,
            #'nan_annotation' : False,
            #'below_label' : None,
            #'above_label' : None,
            #'background_color' : None,
            #'n_colors' : None,
            #'fill' : False,
            #'render' : False,
            #'theme' : None,
            #'unconstrained_font_size' : False
            }
    return args

def mad(c):
    z = c.copy()
    median = np.median(z)
    mad = np.median(np.abs(z - median))
    mod = 0.6745 * np.abs(z - median) / mad
    m = z.copy()
    m[mod > 3.5] = np.sign(z[mod > 3.5]) * (median + 3.5*mad)    
    return m

def colormap(c):
    z = c.flatten(order="F")
    #z = mad(z)
    return z

def decorate(plotter, title, view, c):
    changeBackground(plotter)
    changeView(plotter, view)
    if c is not None:
        moveScalabar(plotter)
    savePdf(plotter,title)

def changeBackground(plotter):
    plotter.background_color = g_window_background

def changeView(plotter, view):
    if view is not None:
        plotter.camera.azimuth = view[0]
        plotter.camera.elevation = view[1]
    else:
        plotter.enable_parallel_projection()
        plotter.view_xy()
        plotter.camera.roll = 180

def savePdf(plotter, title):
    if title and g_save_image:
        plotter.save_graphic(title+'.pdf')

def addScalarbar(plotter, title):
    plotter.add_scalar_bar(
        title=title,
        #n_labels=5,                   # Number of labels to display
        #position_x=0.1,               # X position (normalized from 0 to 1)
        #position_y=0.05,              # Y position (normalized from 0 to 1)
        #width=0.6,                    # Width (normalized)
        #height=0.05,                  # Height (normalized)
        label_font_size=12,           # Font size of labels
        title_font_size=16,           # Font size of title
        shadow=True,                  # Add a shadow
        italic=False,                 # Italic text
        fmt="%.2f",                   # Format string for labels
        interactive=True,             # Allow user to interact with scalar bar
        vertical=True,                # Vertical or horizontal orientation
        color='black',                # Text color
        font_family="arial"           # Font family
        )

def moveScalabar(plotter):
    bar = plotter.scalar_bar
    x, y = bar.GetPosition()
    h = bar.GetHeight()
    bar.SetPosition(x, 0.5 - h/2)

# Hepers ###############################################################

def compare(v1, v2, title=''):
    n1 = v1.size
    n2 = v2.size
    dn = n1 - n2
    if title:
        print(title, end=" : ")
    print(f'{n1} - {n2} = {dn} ({100*dn/n1}%)')

def info(x,y,z,var=''):
    if var:
        print('--------------')
        print(var)
    disp(x, 'x')
    disp(y, 'y')
    disp(z, 'z')

def disp(x,var=''):
    if var:
        print(var, end=': ')
    if x.ndim == 1:
        print(f'shape: [{x.shape[0]}]', end =', ')
        v = x
    elif x.ndim == 2:
        print(f'shape: [{x.shape[0]},{x.shape[1]}]', end=', ')
        v = x.flatten(order='F')
    else:
        print(f"Ops neither vector nor matrix, array has {x.ndim} dimensions!")
        return
    vmin = np.nanmin(v)
    vmax = np.nanmax(v)
    nan = np.isnan(v).any()
    print(f'limits: [{vmin}, {vmax}]', end=', ')
    print(f'width: {vmax - vmin}', end=', ' )
    if x.ndim == 1:
        print(f'resolution: {resolution(v)}', end=', ')
    if nan:
        print(f'has NaN!',end='')
    print('')

def resolution(x):
    v = x[~np.isnan(x)]
    v = np.sort(v)
    dv = np.diff(v)
    return np.min(dv)