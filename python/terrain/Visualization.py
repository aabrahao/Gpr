import Geotiff as gt
import pyvista as pv
import numpy as np
import Dem as dm
import Stl as stl

def decorateView(plotter):
    plotter.camera.azimuth = -45.0
    plotter.camera.elevation = 5
    plotter.background_color = 'white'

def plotDem(database,title='',edges=False,view=None):
    x,y,z = gt.dem(database)
    plotMesh(x,y,z,z,title,edges,view)

def plotMesh(x,y,z,c=None,title='',edges=False,view=None):
    if c is None:
        c = z
    dm.checkIndexing(x,y,z)
    xg, yg = dm.grid(x,y)
    grid = pv.StructuredGrid(*(xg, yg), z)
    cmap = c.flatten(order="F")
    # Plot with colormap
    plotter = pv.Plotter()
    plotter.add_mesh( 
        grid, 
        scalars=cmap, 
        cmap='jet',
        show_scalar_bar=True,
        show_edges=edges,
        scalar_bar_args={
            'title': f'{title}',
            'vertical': False,
            'title_font_size': 14,
            'label_font_size': 10
            }
        )
    if view is not None:
        plotter.camera.azimuth = view[0]
        plotter.camera.elevation = view[1]
    if title:
        plotter.save_graphic(title+'.pdf')
    plotter.show(title=title)

def plotPoints(x,y,z,c=None,title='',view=None):
    if c is None:
        c = z
    point_cloud = pv.PolyData(np.column_stack((x, y, z)))
    plotter = pv.Plotter()
    plotter.add_mesh(point_cloud, 
                     scalars=c, 
                     point_size=8,
                     cmap='jet',
                     render_points_as_spheres=True,
                     scalar_bar_args={
                         'title': f'{title}',
                         'vertical': False,
                         'title_font_size': 12,
                         'label_font_size': 12
                         }
                    )
    if view is not None:
        plotter.camera.azimuth = view[0]
        plotter.camera.elevation = view[1]
    if title:
        plotter.save_graphic(title+'.pdf')
    plotter.show(title=title)

def plotStl(path,view=None):
    mesh = pv.read( stl.filename(path) )
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color='lightgray',
                     show_edges=False)
    if view is not None:
        plotter.camera.azimuth = view[0]
        plotter.camera.elevation = view[1]
    plotter.save_graphic(path+'stl.pdf')
    plotter.show(stl.filename(path))

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
    disp('x', x)
    disp('y', y)
    disp('z', z)

def disp(var, x):
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