import robot.geometry.geotiff as gt
import robot.geometry.dem as dm
import robot.geometry.pointcloud as pc
import robot.geometry.stl as stl

import robot.visualizer.plotter.surface as vz

def generate(path,view, scale, resolution, method, clip):
    # Mesh
    mesh = stl.load(path)
    vz.plotStl(path)
    return
    x,y,z = stl.vertices(mesh)
    vz.info(x,y,z,'Pointcloud')
    # Scale
    # Gazeabo: <scale>0.1 0.1 0.025</scale>
    x = scale[0]*x
    y = scale[1]*y
    z = scale[2]*z
    # Dem
    xs,ys,zs = pc.resample(x,y,z,resolution, method=method)
    xs,ys,zs = dm.shrink(xs,ys,zs,clip)
    # Metrics
    vz.info(xs,ys,zs,'Resampled')
    vz.compare(z,zs)
    vz.plotMesh(xs,ys,zs,zs,edges=True,view=view)
    # Save
    dm.save(xs,ys,zs,path)
    # Check
    database = gt.open(path)
    gt.info(database)
    vz.plotDem(database,view=view)
    # Size
    w = max(xs) - min(xs)
    h = max(ys) - min(ys)
    print('#########################################################')
    print(f"w: {w}")
    print(f"H: {h}")

def main():
    vz.saveImages(False)
    generate('data/mesh/terrains/mars',
              view = (45,5), 
              scale = (1.0,1.0,1.0), 
              resolution = 1.0, 
              method = 'linear', 
              clip = 100)
    #generate('data/mesh/models/part1', 
    #         view = (-45.0,5), 
    #         scale = (1.0,1.0,1.0), 
    #         resolution = 0.01, 
    #         method = 'linear', 
    #         clip = 50)

if __name__ == "__main__":
    main()