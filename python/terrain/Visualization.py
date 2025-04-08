import Geotiff as gt
import pyvista as pv
import numpy as np
import Dem as dm
import Stl as stl

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
            'vertical': True,
            'title_font_size': 14,
            'label_font_size': 10
            }
        )
    decorate(plotter,title,view)
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
                         'vertical': True,
                         'title_font_size': 12,
                         'label_font_size': 12
                         }
                    )
    decorate(plotter,title,view)
    plotter.show(title=title)

def plotStl(path,view=None,edges=False):
    mesh = pv.read( stl.filename(path) )
    plotter = pv.Plotter()
    plotter.add_mesh(mesh, color='lightgray',
                     show_edges=edges)
    changeBackground(plotter)
    changeView(plotter,view)
    savePdf(plotter,path+'stl.pdf')
    plotter.show(stl.filename(path))

# Decoration##############################################################

def decorate(plotter,title,view):
    changeBackground(plotter)
    changeView(plotter,view)
    changeLegend(plotter)
    savePdf(plotter,title)

def changeBackground(plotter):
    plotter.background_color = 'white'

def changeView(plotter, view):
    if view is not None:
        plotter.camera.azimuth = view[0]
        plotter.camera.elevation = view[1]

def changeLegend(plotter):
    bar = plotter.scalar_bar
    x, y = bar.GetPosition()
    #w = bar.GetWidth()
    h = bar.GetHeight()
    bar.SetPosition(x, 0.5 - h/2)

def savePdf(plotter, title):
    if title:
        plotter.save_graphic(title+'.pdf')

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